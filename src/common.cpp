/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    this->declare_parameter("publish_tf", true); // Whether to publish transforms
    this->declare_parameter("visualize_trajectory", true); // Whether to visualize trajectory
    this->declare_parameter("tf_broadcast_rate", 20.0); // Rate for TF broadcasting in Hz
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";
    
    // Get TF broadcasting parameters
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    visualize_trajectory_ = this->get_parameter("visualize_trajectory").as_bool();
    tf_broadcast_rate_ = this->get_parameter("tf_broadcast_rate").as_double();
    
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    // Setup trajectory publisher if visualization is enabled
    if (visualize_trajectory_) {
        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "orb_slam3/trajectory", 10);
    }

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages
    subImuMsgName = "/mono_py_driver/imu_msg"; // Use the topic from the Python driver
    use_imu_ = true; // Enable IMU usage

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    //* subscribe to the IMU messages coming from the Python driver node
    subImuMsg_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    subImuMsgName, 100, std::bind(&MonocularMode::Imu_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", subImuMsgName.c_str());

    // Setup TF broadcasting timer if enabled
    if (publish_tf_) {
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / tf_broadcast_rate_)),
            std::bind(&MonocularMode::broadcast_latest_transform, this));
        RCLCPP_INFO(this->get_logger(), "TF broadcasting enabled at %.1f Hz", tf_broadcast_rate_);
    }
    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        cv::imshow("test_window", cv_ptr->image);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // Get IMU measurements for this frame
    std::vector<ORB_SLAM3::IMU::Point> current_imu_measurements;
    if (use_imu_) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        
        // Get IMU measurements between last frame and current frame
        for (const auto& imu_data : imu_buffer_) {
            if (imu_data.t >= last_image_timestamp_ && imu_data.t <= timeStep) {
                current_imu_measurements.push_back(imu_data);
            }
        }
        
        // Clean up old IMU measurements
        if (!imu_buffer_.empty() && timeStep > 0) {
            auto it = imu_buffer_.begin();
            while (it != imu_buffer_.end() && it->t <= timeStep) {
                ++it;
            }
            imu_buffer_.erase(imu_buffer_.begin(), it);
        }
        
        // Update last image timestamp
        last_image_timestamp_ = timeStep;
    }
    
    // Use TrackMonocular with IMU data if available
    Sophus::SE3f Tcw;
    try {
        RCLCPP_INFO(this->get_logger(), "use_imu_: %d", use_imu_);
        if (use_imu_ && !current_imu_measurements.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Tracking with %zu IMU measurements", 
                       current_imu_measurements.size());
            Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep, current_imu_measurements);
        } else {
            Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep);
        }
        
        // Process the result (existing code)
        if (!Tcw.matrix().isZero(0) && 
            !std::isnan(Tcw.matrix()(0,0)) && 
            !std::isinf(Tcw.matrix()(0,0))) {
            // Convert to world-to-camera transform
            Sophus::SE3f Twc = Tcw.inverse();
        
        // Store the transform for broadcasting
        if (publish_tf_) {
            // Update latest transform (used by broadcast_latest_transform)
            latest_transform_.header.stamp = msg.header.stamp;
            latest_transform_.header.frame_id = "world";
            latest_transform_.child_frame_id = "camera";
            
            // Fill in translation
            latest_transform_.transform.translation.x = Twc.translation().x();
            latest_transform_.transform.translation.y = Twc.translation().y();
            latest_transform_.transform.translation.z = Twc.translation().z();
            
            // Fill in rotation (as quaternion)
            Eigen::Quaternionf q = Twc.unit_quaternion();
            latest_transform_.transform.rotation.x = q.x();
            latest_transform_.transform.rotation.y = q.y();
            latest_transform_.transform.rotation.z = q.z();
            latest_transform_.transform.rotation.w = q.w();
            
            new_transform_available_ = true;
            
            // Also broadcast immediately (in addition to timer-based broadcasts)
            tf_broadcaster_->sendTransform(latest_transform_);
        }
        
        // Update trajectory visualization if enabled
        if (visualize_trajectory_) {
            update_trajectory_visualization(Twc, msg.header.stamp);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Publishing transform: [%.2f, %.2f, %.2f]", 
                    Twc.translation().x(), Twc.translation().y(), Twc.translation().z());
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                               "Tracking failed for this frame - invalid pose");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Exception in TrackMonocular: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Unknown exception in TrackMonocular");
    }
}

void MonocularMode::Imu_callback(const sensor_msgs::msg::Imu& msg) {
    // Skip if IMU is disabled
    if (!use_imu_) return;
    
    // Convert ROS IMU message to ORB-SLAM3 IMU::Point
    // Timestamp is in seconds
    double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    
    ORB_SLAM3::IMU::Point imu_point(
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
        timestamp
    );
    
    // Store IMU measurement in buffer (thread-safe)
    std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
    imu_buffer_.push_back(imu_point);
    
    // Keep buffer from growing too large
    const size_t MAX_IMU_BUFFER = 1000;
    if (imu_buffer_.size() > MAX_IMU_BUFFER) {
        imu_buffer_.erase(imu_buffer_.begin());
    }
}

// Timer callback for TF broadcasting
void MonocularMode::broadcast_latest_transform() {
    if (new_transform_available_) {
        tf_broadcaster_->sendTransform(latest_transform_);
        new_transform_available_ = false;
    }
}

// Update trajectory visualization
void MonocularMode::update_trajectory_visualization(const Sophus::SE3f& Twc, const rclcpp::Time& stamp) {
    // Create new marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;
    marker.ns = "trajectory";
    marker.id = trajectory_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set position from transform
    marker.pose.position.x = Twc.translation().x();
    marker.pose.position.y = Twc.translation().y();
    marker.pose.position.z = Twc.translation().z();
    
    // Set orientation from transform quaternion
    Eigen::Quaternionf q = Twc.unit_quaternion();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    // Set scale and color
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.7;
    marker.color.b = 0.2;
    
    // Keep for the entire session
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 0;
    
    // Add to markers
    trajectory_markers_.markers.push_back(marker);
    
    // Limit size of trajectory visualization
    constexpr size_t MAX_TRAJECTORY_MARKERS = 1000;
    if (trajectory_markers_.markers.size() > MAX_TRAJECTORY_MARKERS) {
        trajectory_markers_.markers.erase(trajectory_markers_.markers.begin());
        
        // Update IDs after removing oldest marker
        for (size_t i = 0; i < trajectory_markers_.markers.size(); ++i) {
            trajectory_markers_.markers[i].id = i;
        }
    }
    
    // Publish trajectory
    trajectory_pub_->publish(trajectory_markers_);
}

