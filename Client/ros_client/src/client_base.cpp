#include "client_base.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>

// SensorNodeBase implementation
SensorNodeBase::SensorNodeBase(const std::string& name, std::shared_ptr<VCCSimClient> client,
                              const std::string& robot_name, double frequency)
    // Ensure unique node names by appending robot_name
    : Node(robot_name + "_" + name), client_(client), robot_name_(robot_name), 
      frequency_(frequency), is_running_(false), sensor_type_(name)
{
    // Create frequency publisher with sensor type in topic name
    freq_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "vccsim/" + name + "/" + robot_name_ + "/frequency", 10);
    
    // Create timer for frequency monitoring
    frequency_monitor_timer_ = this->create_wall_timer(
        2s, std::bind(&SensorNodeBase::monitor_frequency, this));
}

SensorNodeBase::~SensorNodeBase()
{
    stop();
}

void SensorNodeBase::start()
{
    if (!is_running_) {
        is_running_ = true;
        
        // Start the executor thread
        executor_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Starting executor thread for %s", this->get_name());
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->shared_from_this());
            
            while (is_running_ && rclcpp::ok()) {
                executor.spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            
            RCLCPP_INFO(this->get_logger(), "Executor thread for %s stopped", this->get_name());
        });
    }
}

void SensorNodeBase::stop()
{
    if (is_running_) {
        is_running_ = false;
        
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }
}

void SensorNodeBase::update_frequency_stats()
{
    auto now = std::chrono::steady_clock::now();
    
    std::lock_guard<std::mutex> lock(frequency_stats_.stats_mutex);
    
    frequency_stats_.publish_count++;
    
    if (frequency_stats_.first_publish_time == std::chrono::steady_clock::time_point{}) {
        // First publish event
        frequency_stats_.first_publish_time = now;
        frequency_stats_.last_publish_time = now;
    } else {
        frequency_stats_.last_publish_time = now;
        
        // Calculate total elapsed time since first publish
        auto total_elapsed = now - frequency_stats_.first_publish_time;
        auto total_seconds = std::chrono::duration<double>(total_elapsed).count();
        
        // Average frequency = total publications / total time
        if (total_seconds > 0) {
            frequency_stats_.current_frequency = (frequency_stats_.publish_count - 1) / total_seconds;
        }
    }
}

void SensorNodeBase::monitor_frequency()
{
    std::lock_guard<std::mutex> lock(frequency_stats_.stats_mutex);
    
    // Include sensor type and robot name in log message for clear identification
    RCLCPP_INFO(this->get_logger(), "[%s][%s] frequency: %.2f Hz", 
                sensor_type_.c_str(), robot_name_.c_str(), 
                frequency_stats_.current_frequency);
    
    std_msgs::msg::Float32 freq_msg;
    freq_msg.data = static_cast<float>(frequency_stats_.current_frequency);
    freq_pub_->publish(freq_msg);
}

// RGB Camera Node implementation
RGBCameraNode::RGBCameraNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name,
                           int camera_index, double frequency)
    : SensorNodeBase("rgb_camera", client, robot_name, frequency), camera_index_(camera_index)
{
    // Create publishers with robot name in topic path
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vccsim/" + robot_name + "/rgb/image", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "vccsim/" + robot_name + "/rgb/camera_info", 10);
    
    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&RGBCameraNode::publish_data, this));
}

void RGBCameraNode::publish_data()
{
    try {
        // Get RGB image data
        VCCSim::RGBCameraImageData rgb_data = 
            client_->GetRGBIndexedCameraImageData(robot_name_, camera_index_, VCCSim::PNG);
        
        // Convert to ROS message
        auto ros_image = convert_to_ros_image(rgb_data);
        
        // Create camera info message
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = ros_image.header;
        camera_info.width = rgb_data.width();
        camera_info.height = rgb_data.height();
        
        // Publish
        image_pub_->publish(ros_image);
        camera_info_pub_->publish(camera_info);
        
        // Update frequency statistics
        update_frequency_stats();
        
        RCLCPP_DEBUG(this->get_logger(), "Published RGB data, image size: %dx%d", 
                    rgb_data.width(), rgb_data.height());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing RGB data: %s", e.what());
    }
}

sensor_msgs::msg::Image RGBCameraNode::convert_to_ros_image(const VCCSim::RGBCameraImageData& vcc_image)
{
    sensor_msgs::msg::Image ros_image;
    ros_image.header.stamp = this->now();
    ros_image.header.frame_id = "rgb_camera_frame";
    
    if (vcc_image.format() == VCCSim::PNG || vcc_image.format() == VCCSim::JPEG) {
        // For compressed formats, we need to decompress
        try {
            // Convert to cv::Mat first
            std::vector<uchar> img_data(vcc_image.data().begin(), vcc_image.data().end());
            cv::Mat cv_img = cv::imdecode(cv::Mat(img_data), cv::IMREAD_COLOR);
            
            // Then convert to ROS image using cv_bridge
            cv_bridge::CvImage cv_bridge_img;
            cv_bridge_img.header.stamp = this->now();
            cv_bridge_img.header.frame_id = "rgb_camera_frame";
            cv_bridge_img.encoding = "bgr8";  // OpenCV uses BGR
            cv_bridge_img.image = cv_img;
            
            cv_bridge_img.toImageMsg(ros_image);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    } else if (vcc_image.format() == VCCSim::RAW) {
        // For raw format
        ros_image.encoding = "rgb8";
        ros_image.is_bigendian = false;
        ros_image.step = 3 * vcc_image.width();
        ros_image.height = vcc_image.height();
        ros_image.width = vcc_image.width();
        
        // Make sure the data size matches the expected size
        size_t expected_size = ros_image.step * ros_image.height;
        if (vcc_image.data().size() == expected_size) {
            ros_image.data.assign(vcc_image.data().begin(), vcc_image.data().end());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Raw image data size mismatch: expected %zu, got %zu", 
                         expected_size, vcc_image.data().size());
        }
    }
    
    return ros_image;
}

// Depth Camera Node implementation
DepthCameraNode::DepthCameraNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name,
                               double frequency)
    : SensorNodeBase("depth_camera", client, robot_name, frequency)
{
    // Create publishers with robot name in topic path
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vccsim/" + robot_name + "/depth/image", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "vccsim/" + robot_name + "/depth/camera_info", 10);
    
    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&DepthCameraNode::publish_data, this));
}

void DepthCameraNode::publish_data()
{
    try {
        // Get depth image size
        auto [width, height] = client_->GetDepthCameraImageSize(robot_name_);
        
        // Get depth image data
        std::vector<float> depth_data = client_->GetDepthCameraImageData(robot_name_);
        
        // Convert to ROS message
        auto ros_image = convert_to_ros_depth_image(depth_data, width, height);
        
        // Create camera info message
        sensor_msgs::msg::CameraInfo camera_info;
        camera_info.header = ros_image.header;
        camera_info.width = width;
        camera_info.height = height;
        
        // Publish
        image_pub_->publish(ros_image);
        camera_info_pub_->publish(camera_info);
        
        // Update frequency statistics
        update_frequency_stats();
        
        RCLCPP_DEBUG(this->get_logger(), "Published depth data, image size: %dx%d", width, height);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing depth data: %s", e.what());
    }
}

sensor_msgs::msg::Image DepthCameraNode::convert_to_ros_depth_image(const std::vector<float>& depth_data, 
                                                                  int width, int height)
{
    sensor_msgs::msg::Image ros_image;
    ros_image.header.stamp = this->now();
    ros_image.header.frame_id = "depth_camera_frame";
    ros_image.height = height;
    ros_image.width = width;
    ros_image.encoding = "32FC1"; // 32-bit float, 1 channel
    ros_image.is_bigendian = false;
    ros_image.step = width * sizeof(float);
    
    // Copy data
    size_t data_size = depth_data.size() * sizeof(float);
    ros_image.data.resize(data_size);
    memcpy(ros_image.data.data(), depth_data.data(), data_size);
    
    return ros_image;
}

// LiDAR Node implementation
LidarNode::LidarNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name,
                   double frequency)
    : SensorNodeBase("lidar", client, robot_name, frequency)
{
    // Create publishers with robot name in topic path
    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "vccsim/" + robot_name + "/lidar/points", 10);
    
    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&LidarNode::publish_data, this));
}


void LidarNode::publish_data()
{
    try {
        // Get LiDAR data
        std::vector<VCCSim::Point> lidar_points = client_->GetLidarData(robot_name_);
        
        // Convert to ROS message
        auto ros_point_cloud = convert_to_ros_pointcloud(lidar_points);
        
        // Publish
        points_pub_->publish(ros_point_cloud);
        
        // Update frequency statistics
        update_frequency_stats();
        
        RCLCPP_DEBUG(this->get_logger(), "Published LiDAR data, points: %zu", lidar_points.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing LiDAR data: %s", e.what());
    }
}

sensor_msgs::msg::PointCloud2 LidarNode::convert_to_ros_pointcloud(const std::vector<VCCSim::Point>& vcc_points)
{
    sensor_msgs::msg::PointCloud2 ros_cloud;
    
    // Set header
    ros_cloud.header.stamp = this->now();
    ros_cloud.header.frame_id = "lidar_frame";
    
    // Set fields
    ros_cloud.fields.resize(3);
    ros_cloud.fields[0].name = "x";
    ros_cloud.fields[0].offset = 0;
    ros_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_cloud.fields[0].count = 1;
    
    ros_cloud.fields[1].name = "y";
    ros_cloud.fields[1].offset = 4;
    ros_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_cloud.fields[1].count = 1;
    
    ros_cloud.fields[2].name = "z";
    ros_cloud.fields[2].offset = 8;
    ros_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    ros_cloud.fields[2].count = 1;
    
    // Set properties
    ros_cloud.point_step = 12; // 3 * sizeof(float)
    ros_cloud.row_step = ros_cloud.point_step * vcc_points.size();
    ros_cloud.height = 1;
    ros_cloud.width = vcc_points.size();
    ros_cloud.is_dense = false;
    
    // Allocate memory for data
    ros_cloud.data.resize(ros_cloud.row_step);
    
    // Copy data
    float* data_ptr = reinterpret_cast<float*>(ros_cloud.data.data());
    for (const auto& point : vcc_points) {
        *data_ptr++ = point.x();
        *data_ptr++ = point.y();
        *data_ptr++ = point.z();
    }
    
    return ros_cloud;
}

// Drone Pose Node implementation
DronePoseNode::DronePoseNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name,
                           double frequency)
    : SensorNodeBase("drone_pose", client, robot_name, frequency)
{
    // Create publishers with robot name in topic path
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "vccsim/" + robot_name + "/drone/pose", 10);
    
    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
        std::bind(&DronePoseNode::publish_data, this));
}


void DronePoseNode::publish_data()
{
    try {
        // Get drone pose
        VCCSim::Pose drone_pose = client_->GetDronePose(robot_name_);
        
        // Convert to ROS message
        auto ros_pose = convert_to_ros_pose(drone_pose);
        
        // Publish
        pose_pub_->publish(ros_pose);
        
        // Update frequency statistics
        update_frequency_stats();
        
        RCLCPP_DEBUG(this->get_logger(), "Published drone pose: x=%.2f, y=%.2f, z=%.2f",
                    drone_pose.x(), drone_pose.y(), drone_pose.z());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error publishing drone pose: %s", e.what());
    }
}

geometry_msgs::msg::PoseStamped DronePoseNode::convert_to_ros_pose(const VCCSim::Pose& vcc_pose)
{
    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header.stamp = this->now();
    ros_pose.header.frame_id = "base_link";
    
    ros_pose.pose.position.x = vcc_pose.x();
    ros_pose.pose.position.y = vcc_pose.y();
    ros_pose.pose.position.z = vcc_pose.z();
    
    // Convert roll, pitch, yaw to quaternion
    double roll = vcc_pose.roll();
    double pitch = vcc_pose.pitch();
    double yaw = vcc_pose.yaw();
    
    // Simple conversion using sin/cos/2
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    ros_pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    ros_pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    ros_pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    ros_pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;
    
    return ros_pose;
}

// Node Manager implementation
VCCSimNodeManager::VCCSimNodeManager() : is_running_(false)
{
}

VCCSimNodeManager::~VCCSimNodeManager()
{
    stop();
}

void VCCSimNodeManager::init(int argc, char* argv[])
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create parameter node with a unique name
    param_node_ = std::make_shared<rclcpp::Node>("vccsim_param_node_" + 
                    std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    
    // Declare and get parameters
    param_node_->declare_parameter("vccsim_host", "localhost");
    param_node_->declare_parameter("vccsim_port", 50996);
    param_node_->declare_parameter("robot_name", "Mavic");
    param_node_->declare_parameter("rgb_camera_index", 0);
    
    param_node_->declare_parameter("rgb_active", true);
    param_node_->declare_parameter("depth_active", true);
    param_node_->declare_parameter("lidar_active", true);
    param_node_->declare_parameter("drone_pose_active", true);
    
    param_node_->declare_parameter("rgb_frequency", 5.0);
    param_node_->declare_parameter("depth_frequency", 5.0);
    param_node_->declare_parameter("lidar_frequency", 5.0);
    param_node_->declare_parameter("drone_pose_frequency", 30.0);
    
    // Get connection parameters
    std::string host = param_node_->get_parameter("vccsim_host").as_string();
    int port = param_node_->get_parameter("vccsim_port").as_int();
    robot_name_ = param_node_->get_parameter("robot_name").as_string();
    rgb_camera_index_ = param_node_->get_parameter("rgb_camera_index").as_int();
    
    // Get component activation flags
    rgb_active_ = param_node_->get_parameter("rgb_active").as_bool();
    depth_active_ = param_node_->get_parameter("depth_active").as_bool();
    lidar_active_ = param_node_->get_parameter("lidar_active").as_bool();
    drone_pose_active_ = param_node_->get_parameter("drone_pose_active").as_bool();
    
    // Get frequency parameters
    rgb_frequency_ = param_node_->get_parameter("rgb_frequency").as_double();
    depth_frequency_ = param_node_->get_parameter("depth_frequency").as_double();
    lidar_frequency_ = param_node_->get_parameter("lidar_frequency").as_double();
    drone_pose_frequency_ = param_node_->get_parameter("drone_pose_frequency").as_double();
    
    // Initialize VCCSimClient (shared between all nodes)
    client_ = std::make_shared<VCCSimClient>(host, port);
    
    // Create sensor nodes if active - each with unique node name
    if (rgb_active_) {
        rgb_node_ = std::make_shared<RGBCameraNode>(client_, robot_name_, rgb_camera_index_, rgb_frequency_);
    }
    
    if (depth_active_) {
        depth_node_ = std::make_shared<DepthCameraNode>(client_, robot_name_, depth_frequency_);
    }
    
    if (lidar_active_) {
        lidar_node_ = std::make_shared<LidarNode>(client_, robot_name_, lidar_frequency_);
    }
    
    if (drone_pose_active_) {
        drone_node_ = std::make_shared<DronePoseNode>(client_, robot_name_, drone_pose_frequency_);
    }
    
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(param_node_);
    tf_timer_ = param_node_->create_wall_timer(
        std::chrono::milliseconds(1), 
        std::bind(&VCCSimNodeManager::publish_transforms, this));

    RCLCPP_INFO(param_node_->get_logger(), "VCCSimNodeManager initialized with host: %s, port: %d", 
                host.c_str(), port);
}

void VCCSimNodeManager::start()
{
    if (!is_running_) {
        is_running_ = true;
        
        // Start each node in its own thread
        if (rgb_node_) rgb_node_->start();
        if (depth_node_) depth_node_->start();
        if (lidar_node_) lidar_node_->start();
        if (drone_node_) drone_node_->start();

        if (param_node_) {
            RCLCPP_INFO(param_node_->get_logger(), "Starting TF broadcaster node");
            std::thread([this]() {
                rclcpp::executors::SingleThreadedExecutor executor;
                executor.add_node(param_node_);
                
                while (is_running_ && rclcpp::ok()) {
                    executor.spin_some();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                
                RCLCPP_INFO(param_node_->get_logger(), "TF broadcaster node stopped");
            }).detach();  // Use detach for simplicity, but consider managing this thread
        }
    }
}

void VCCSimNodeManager::stop()
{
    if (is_running_) {
        is_running_ = false;
        
        // Stop each node
        if (rgb_node_) rgb_node_->stop();
        if (depth_node_) depth_node_->stop();
        if (lidar_node_) lidar_node_->stop();
        if (drone_node_) drone_node_->stop();

        if (tf_timer_) {
            tf_timer_->cancel();
        }

        // Note: The param_node thread will exit based on is_running_ flag
        // Wait a moment for threads to clean up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void VCCSimNodeManager::publish_transforms()
{
    // Get drone pose
    VCCSim::Pose drone_pose = client_->GetDronePose(robot_name_);
    
    auto now = rclcpp::Clock().now();
    
    // Create transforms
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    
    geometry_msgs::msg::TransformStamped drone_transform;
    drone_transform.header.stamp = now;
    drone_transform.header.frame_id = "map";
    drone_transform.child_frame_id = "base_link";
    
    drone_transform.transform.translation.x = drone_pose.x();
    drone_transform.transform.translation.y = drone_pose.y();
    drone_transform.transform.translation.z = drone_pose.z();
    
    double roll = drone_pose.roll();
    double pitch = drone_pose.pitch();
    double yaw = drone_pose.yaw();

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    drone_transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    drone_transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    drone_transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    drone_transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
        
    transforms.push_back(drone_transform);
    
    // Drone -> Lidar transform
    geometry_msgs::msg::TransformStamped lidar_transform;
    lidar_transform.header.stamp = now;
    lidar_transform.header.frame_id = "base_link";
    lidar_transform.child_frame_id = "lidar_frame";
    
    // Identity transform for simple case
    lidar_transform.transform.translation.x = 0.0;
    lidar_transform.transform.translation.y = 0.0;
    lidar_transform.transform.translation.z = 0.0;
    lidar_transform.transform.rotation.w = 1.0;
    
    transforms.push_back(lidar_transform);
    
    // rgb_camera -> base_link transform
    geometry_msgs::msg::TransformStamped rgb_transform;
    rgb_transform.header.stamp = now;
    rgb_transform.header.frame_id = "base_link";
    rgb_transform.child_frame_id = "rgb_camera_frame";

    // Identity transform for simple case
    rgb_transform.transform.translation.x = 0.0;
    rgb_transform.transform.translation.y = 0.0;
    rgb_transform.transform.translation.z = 0.0;
    rgb_transform.transform.rotation.w = 1.0;

    transforms.push_back(rgb_transform);

    // depth_camera -> base_link transform
    geometry_msgs::msg::TransformStamped depth_transform;
    depth_transform.header.stamp = now;
    depth_transform.header.frame_id = "base_link";
    depth_transform.child_frame_id = "depth_camera_frame";

    // Identity transform for simple case
    depth_transform.transform.translation.x = 0.0;
    depth_transform.transform.translation.y = 0.0;
    depth_transform.transform.translation.z = 0.0;
    depth_transform.transform.rotation.w = 1.0;

    transforms.push_back(depth_transform);

    tf_broadcaster_->sendTransform(transforms);
}

int main(int argc, char* argv[])
{
    VCCSimNodeManager manager;
    manager.init(argc, argv);
    manager.start();
    
    // Create a unique spin node
    auto spin_node_name = "vccsim_spin_node_" + 
                        std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    auto spin_node = std::make_shared<rclcpp::Node>(spin_node_name);
    rclcpp::spin(spin_node);
    
    // Stop all nodes
    manager.stop();
    
    return 0;
}