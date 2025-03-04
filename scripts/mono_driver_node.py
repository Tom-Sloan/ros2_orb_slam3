#!/usr/bin/env python3


"""
Python node for the MonocularMode cpp node.

Author: Azmyin Md. Kamal
Date: 01/01/2024

Requirements
* Dataset must be configured in EuRoC MAV format
* Paths to dataset must be set before bulding (or running) this node
* Make sure to set path to your workspace in common.hpp

Command line arguments
-- settings_name: EuRoC, TUM2, KITTI etc; the name of the .yaml file containing camera intrinsics and other configurations
-- image_seq: MH01, V102, etc; the name of the image sequence you want to run

"""

# Imports
#* Import Python modules
import sys # System specific modules
import os # Operating specific functions
import glob
import time # Python timing module
import copy # For deepcopying arrays
import shutil # High level folder operation tool
from pathlib import Path # To find the "home" directory location
import argparse # To accept user arguments from commandline
import natsort # To ensure all images are chosen loaded in the correct order
import yaml # To manipulate YAML files for reading configuration files
import copy # For making deepcopies of openCV matrices, python lists, numpy arrays etc.
import numpy as np # Python Linear Algebra module
import cv2 # OpenCV

#* ROS2 imports
import ament_index_python.packages
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# If you have more files in the submodules folder
# from .submodules.py_utils import fn1 # Import helper functions from files in your submodules folder

# Import a custom message interface
# from your_custom_msg_interface.msg import CustomMsg #* Note the camel caps convention

# Import ROS2 message templates
from sensor_msgs.msg import Image # http://wiki.ros.org/sensor_msgs
from std_msgs.msg import String, Float64 # ROS2 string message template
from cv_bridge import CvBridge, CvBridgeError # Library to convert image messages to numpy array

#* Class definition
class MonoDriver(Node):
    def __init__(self, node_name = "mono_py_node"):
        super().__init__(node_name) # Initializes the rclpy.Node class. It expects the name of the node

        # Initialize parameters to be passed from the command line (or launch file)
        self.declare_parameter("settings_name","EuRoC")
        self.declare_parameter("image_seq","NULL")

        #* Parse values sent by command line
        self.settings_name = str(self.get_parameter('settings_name').value) 
        self.image_seq = str(self.get_parameter('image_seq').value)

        # DEBUG
        print(f"-------------- Received parameters --------------------------\n")
        print(f"self.settings_name: {self.settings_name}")
        print(f"self.image_seq: {self.image_seq}")
        print()

        # Global path definitions
        self.home_dir = "/home/sam3/Desktop/Toms_Workspace/active_mapping/src/ros2_orb_slam3" #! Change this to match path to your workspace
        self.parent_dir = "TEST_DATASET" #! Change or provide path to the parent directory where data for all image sequences are stored
        self.image_sequence_dir = self.home_dir + "/" + self.parent_dir + "/" + self.image_seq # Full path to the image sequence folder

        print(f"self.image_sequence_dir: {self.image_sequence_dir}\n")

        # Global variables
        self.node_name = "mono_py_driver"
        self.image_seq_dir = ""
        self.imgz_seqz = []
        self.time_seqz = [] # Maybe redundant

        # Define a CvBridge object
        self.br = CvBridge()

        # Read images from the chosen dataset, order them in ascending order and prepare timestep data as well
        self.imgz_seqz_dir, self.imgz_seqz, self.time_seqz = self.get_image_dataset_asl(self.image_sequence_dir, "mav0") 

        print(self.image_seq_dir)
        print(len(self.imgz_seqz))

        #* ROS2 publisher/subscriber variables [HARDCODED]
        self.pub_exp_config_name = "/mono_py_driver/experiment_settings" 
        self.sub_exp_ack_name = "/mono_py_driver/exp_settings_ack"
        self.pub_img_to_agent_name = "/mono_py_driver/img_msg"
        self.pub_timestep_to_agent_name = "/mono_py_driver/timestep_msg"
        self.pub_imu_to_agent_name = "/imu/data"  # Default IMU topic that matches launch file
        self.send_config = True # Set False once handshake is completed with the cpp node
        self.use_imu = False  # Whether to publish IMU data (if available)
        
        # Check if IMU data directory exists
        agent_imu_fld = self.image_sequence_dir + "/mav0/imu0"
        self.imu_data_path = agent_imu_fld + "/data.csv" if os.path.exists(agent_imu_fld) else None
        if self.imu_data_path and os.path.exists(self.imu_data_path):
            self.use_imu = True
            self.imu_data = self.load_imu_data(self.imu_data_path)
            print(f"Found IMU data at: {self.imu_data_path}")
        else:
            print("No IMU data found. Running in image-only mode.")
        
        #* Setup ROS2 publishers and subscribers
        self.publish_exp_config_ = self.create_publisher(String, self.pub_exp_config_name, 1) # Publish configs to the ORB-SLAM3 C++ node

        #* Build the configuration string to be sent out
        #self.exp_config_msg = self.settings_name + "/" + self.image_seq # Example EuRoC/sample_euroc_MH05
        self.exp_config_msg = self.settings_name # Example EuRoC
        print(f"Configuration to be sent: {self.exp_config_msg}")


        #* Subscriber to get acknowledgement from CPP node that it received experimetn settings
        self.subscribe_exp_ack_ = self.create_subscription(String, 
                                                           self.sub_exp_ack_name, 
                                                           self.ack_callback ,10)
        self.subscribe_exp_ack_

        # Publisher to send RGB image
        self.publish_img_msg_ = self.create_publisher(Image, self.pub_img_to_agent_name, 1)
        
        # Publisher for timestamp and IMU data
        self.publish_timestep_msg_ = self.create_publisher(Float64, self.pub_timestep_to_agent_name, 1)
        
        # Initialize IMU publisher if IMU data is available
        if self.use_imu:
            from sensor_msgs.msg import Imu
            self.publish_imu_msg_ = self.create_publisher(Imu, self.pub_imu_to_agent_name, 10)


        # Initialize work variables for main logic
        self.start_frame = 0 # Default 0
        self.end_frame = -1 # Default -1
        self.frame_stop = -1 # Set -1 to use the whole sequence, some positive integer to force sequence to stop, 350 test2, 736 test3
        self.show_imgz = False # Default, False, set True to see the output directly from this node
        self.frame_id = 0 # Integer id of an image frame
        self.frame_count = 0 # Ensure we are consistent with the count number of the frame
        self.inference_time = [] # List to compute average time

        print()
        print(f"MonoDriver initialized, attempting handshake with CPP node")
    # ****************************************************************************************

    # ****************************************************************************************
    def get_image_dataset_asl(self, exp_dir, agent_name = "mav0"):
        """
            Returns images and list of timesteps in ascending order from a ASL formatted dataset
        """
        
        # Define work variables
        imgz_file_list = []
        time_list = []

        #* Only works for EuRoC MAV format
        agent_cam0_fld = exp_dir + "/" + agent_name + "/" + "cam0"
        imgz_file_dir = agent_cam0_fld + "/" + "data" + "/"
        imgz_file_list = natsort.natsorted(os.listdir(imgz_file_dir),reverse=False)
        # print(len(img_file_list)) # Debug, checks the number of rgb images

        # Extract timesteps from image names
        for iox in imgz_file_list:
            time_step = iox.split(".")[0]
            time_list.append(time_step)
            #print(time_step)

        return imgz_file_dir, imgz_file_list, time_list
    # ****************************************************************************************
    
    # ****************************************************************************************
    def load_imu_data(self, imu_file_path):
        """
        Load IMU data from EuRoC format CSV file
        Format: timestamp [ns], w_x [rad/s], w_y [rad/s], w_z [rad/s], a_x [m/s^2], a_y [m/s^2], a_z [m/s^2]
        """
        imu_data = {}
        try:
            with open(imu_file_path, 'r') as f:
                # Skip header line
                next(f)
                for line in f:
                    values = line.strip().split(',')
                    if len(values) == 7:
                        timestamp = float(values[0])  # nanoseconds
                        # Store angular velocity (w) and linear acceleration (a)
                        imu_data[timestamp] = {
                            'w': [float(values[1]), float(values[2]), float(values[3])],
                            'a': [float(values[4]), float(values[5]), float(values[6])]
                        }
            print(f"Loaded {len(imu_data)} IMU measurements")
            return imu_data
        except Exception as e:
            print(f"Error loading IMU data: {e}")
            return {}
    # ****************************************************************************************

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
            Callback function
        """
        print(f"Got ack: {msg.data}")
        
        if(msg.data == "ACK"):
            self.send_config = False
            # self.subscribe_exp_ack_.destory() # TODO doesn't work 
    # ****************************************************************************************
    
    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            # print(f"Sent mesasge: {self.exp_config_msg}")
            msg = String()
            msg.data = self.exp_config_msg
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************
    
    # ****************************************************************************************
    def run_py_node(self, idx, imgz_name):
        """
            Master function that sends the RGB image message to the CPP node
            Also sends IMU data if available
        """

        # Initialize work variables
        img_msg = None # sensor_msgs image object

        # Path to this image
        img_look_up_path = self.imgz_seqz_dir + imgz_name
        timestep = float(imgz_name.split(".")[0]) # Timestamp in nanoseconds
        self.frame_id = self.frame_id + 1  
        #print(img_look_up_path)
        # print(f"Frame ID: {frame_id}")

        # Create and publish image message
        img_msg = self.br.cv2_to_imgmsg(cv2.imread(img_look_up_path), encoding="passthrough")
        timestep_msg = Float64()
        timestep_msg.data = timestep

        # Publish corresponding IMU data if available
        if self.use_imu and hasattr(self, 'publish_imu_msg_'):
            self.publish_imu_data(timestep)
            
        # Publish RGB image and timestep, must be in the order shown below
        try:
            self.publish_timestep_msg_.publish(timestep_msg) 
            self.publish_img_msg_.publish(img_msg)
        except CvBridgeError as e:
            print(e)
    # ****************************************************************************************
    
    # ****************************************************************************************
    def publish_imu_data(self, current_timestamp):
        """
        Publish IMU data corresponding to the current image timestamp
        For simplicity, publishes the closest IMU measurement to the current image timestamp
        """
        if not self.imu_data:
            return
            
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import Vector3
        from rclpy.time import Time
        
        # Find the closest IMU measurement to the current image timestamp
        closest_timestamp = min(self.imu_data.keys(), key=lambda x: abs(x - current_timestamp))
        imu_measurement = self.imu_data[closest_timestamp]
        
        # Create IMU message
        imu_msg = Imu()
        # Set header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu"
        
        # Set angular velocity (rad/s)
        imu_msg.angular_velocity.x = imu_measurement['w'][0]
        imu_msg.angular_velocity.y = imu_measurement['w'][1]
        imu_msg.angular_velocity.z = imu_measurement['w'][2]
        
        # Set linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = imu_measurement['a'][0]
        imu_msg.linear_acceleration.y = imu_measurement['a'][1]
        imu_msg.linear_acceleration.z = imu_measurement['a'][2]
        
        # Set default covariance (replace with actual values if available)
        # -1 indicates unknown covariance
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # Publish IMU message
        self.publish_imu_msg_.publish(imu_msg)
    # ****************************************************************************************
        

# main function
def main(args = None):
    rclpy.init(args=args) # Initialize node
    n = MonoDriver("mono_py_node") #* Initialize the node
    rate = n.create_rate(20) # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    
    #* Blocking loop to initialize handshake
    while(n.send_config == True):
        n.handshake_with_cpp_node()
        rclpy.spin_once(n)
        #self.rate.sleep(10) # Potential bug, breaks code

        if(n.send_config == False):
            break
        
    print(f"Handshake complete")

    #* Blocking loop to send RGB image and timestep message
    for idx, imgz_name in enumerate(n.imgz_seqz[n.start_frame:n.end_frame]):
        try:
            rclpy.spin_once(n) # Blocking we need a non blocking take care of callbacks
            n.run_py_node(idx, imgz_name)
            rate.sleep()

            # DEBUG, if you want to halt sending images after a certain Frame is reached
            if (n.frame_id>n.frame_stop and n.frame_stop != -1):
                print(f"BREAK!")
                break
        
        except KeyboardInterrupt:
            break

    # Cleanup
    cv2.destroyAllWindows() # Close all image windows
    n.destroy_node() # Release all resource related to this node
    rclpy.shutdown()

# Dunders, this .py is the main file
if __name__=="__main__":
    main()
