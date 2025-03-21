#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"  // For LiDAR data
#include "geometry_msgs/msg/pose_stamped.hpp"  // For drone pose
#include "sensor_msgs/point_cloud2_iterator.hpp"  // For creating PointCloud2 messages
#include "tf2_ros/transform_broadcaster.h"  // For publishing transforms
#include "tf2_ros/static_transform_broadcaster.h"  // For static transforms
#include "geometry_msgs/msg/transform_stamped.hpp"  // For transform messages
#include "tf2/LinearMath/Quaternion.h"  // For quaternion conversions

#include "VCCSimClient.h"

using namespace std::chrono_literals;

// Convert from left-handed (UE) to right-handed (ROS) coordinate system and cm to m
VCCTypes::Point transform_point(const VCCTypes::Point& point) {
  VCCTypes::Point transformed;
  // X remains the same, Y gets negated, and convert from cm to m
  transformed.x = point.x / 100.0f;  // cm to m
  transformed.y = -point.y / 100.0f; // negate Y and cm to m 
  transformed.z = point.z / 100.0f;  // cm to m
  return transformed;
}

// Convert position from left-handed (UE) to right-handed (ROS) and cm to m
geometry_msgs::msg::Point transform_position(const VCCTypes::Pose& pose) {
  geometry_msgs::msg::Point position;
  position.x = pose.x / 100.0f;    // cm to m
  position.y = -pose.y / 100.0f;   // negate Y and cm to m
  position.z = pose.z / 100.0f;    // cm to m
  return position;
}

// Convert rotation from left-handed (UE) to right-handed (ROS)
tf2::Quaternion transform_rotation(const VCCTypes::Pose& pose) {
  // In left-handed to right-handed conversion, we need to:
  // 1. Negate both the rotation angle and axis for Y
  // 2. Negate both the rotation angle and axis for Z (or negate just Z yaw)
  
  // Convert angles to radians if they're in degrees
  float roll_rad = pose.roll; // Assuming already in radians
  float pitch_rad = pose.pitch; // Assuming already in radians 
  float yaw_rad = -pose.yaw; // Negate yaw for left-handed to right-handed conversion
  
  // Create quaternion with modified angles
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  q.normalize();
  return q;
}

class VCCSimROS2Node : public rclcpp::Node {
private:
  // VCCSimClient instance
  std::unique_ptr<VCCSimClient> client_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr drone_pose_pub_;
  
  // TF broadcasters
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  // Timers for each component
  rclcpp::TimerBase::SharedPtr rgb_timer_;
  rclcpp::TimerBase::SharedPtr depth_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  rclcpp::TimerBase::SharedPtr drone_pose_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  
  // Camera index
  int rgb_camera_index_;
  
  // Component activation flags
  bool rgb_active_;
  bool depth_active_;
  bool lidar_active_;
  bool drone_pose_active_;
  
  // Image dimensions (to be determined from the actual data)
  int rgb_width_ = 640;
  int rgb_height_ = 480;
  int depth_width_ = 640;
  int depth_height_ = 480;

public:
  VCCSimROS2Node() : Node("vccsim_ros2_node") {
    // Declare and get connection parameters
    this->declare_parameter<std::string>("vccsim_host", "172.31.178.18");
    this->declare_parameter<int>("vccsim_port", 50996);
    this->declare_parameter<std::string>("robot_name", "Mavic");
    this->declare_parameter<int>("rgb_camera_index", 0);
    this->declare_parameter<std::string>("frame_id", "map");
    
    // Declare component activation parameters
    this->declare_parameter<bool>("rgb_active", true);
    this->declare_parameter<bool>("depth_active", true);
    this->declare_parameter<bool>("lidar_active", true);
    this->declare_parameter<bool>("drone_pose_active", true);
    
    // Declare frequency parameters (Hz)
    this->declare_parameter<double>("rgb_frequency", 5.0);
    this->declare_parameter<double>("depth_frequency", 5.0);
    this->declare_parameter<double>("lidar_frequency", 10.0);
    this->declare_parameter<double>("drone_pose_frequency", 20.0);
    this->declare_parameter<double>("tf_frequency", 30.0);
    
    // Get connection parameters
    std::string host = this->get_parameter("vccsim_host").as_string();
    int port = this->get_parameter("vccsim_port").as_int();
    rgb_camera_index_ = this->get_parameter("rgb_camera_index").as_int();
    
    // Get component activation parameters
    rgb_active_ = this->get_parameter("rgb_active").as_bool();
    depth_active_ = this->get_parameter("depth_active").as_bool();
    lidar_active_ = this->get_parameter("lidar_active").as_bool();
    drone_pose_active_ = this->get_parameter("drone_pose_active").as_bool();
    
    // Get frequency parameters
    double rgb_freq = this->get_parameter("rgb_frequency").as_double();
    double depth_freq = this->get_parameter("depth_frequency").as_double();
    double lidar_freq = this->get_parameter("lidar_frequency").as_double();
    double drone_pose_freq = this->get_parameter("drone_pose_frequency").as_double();
    double tf_freq = this->get_parameter("tf_frequency").as_double();
    
    // Convert frequencies to durations
    auto rgb_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rgb_freq));
    auto depth_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / depth_freq));
    auto lidar_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / lidar_freq));
    auto drone_pose_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / drone_pose_freq));
    auto tf_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / tf_freq));
    
    // Initialize the VCCSimClient
    client_ = std::make_unique<VCCSimClient>(host, port);
    
    // Initialize TF broadcasters
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    
    // Create publishers (only if the respective component is active)
    if (rgb_active_) {
      rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vccsim/rgb_image", 10);
      rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "vccsim/rgb_camera_info", 10);
        
      // Try to get the actual image dimensions
      std::string robot_name = this->get_parameter("robot_name").as_string();
      try {
        auto dimensions = client_->GetRGBCameraImageSize(robot_name, rgb_camera_index_);
        rgb_width_ = std::get<0>(dimensions);
        rgb_height_ = std::get<1>(dimensions);
        RCLCPP_INFO(this->get_logger(), "Got RGB image dimensions: %dx%d", rgb_width_, rgb_height_);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not get RGB image dimensions: %s", e.what());
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not get RGB image dimensions: %s", e.what());
      }
    }
    
    if (depth_active_) {
      depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "vccsim/depth_image", 10);
      depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "vccsim/depth_camera_info", 10);
    }
    
    if (lidar_active_) {
      lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "vccsim/lidar_points", 10);
    }
    
    if (drone_pose_active_) {
      drone_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "vccsim/drone_pose", 10);
    }
    
    // Create timers (only if the respective component is active)
    if (rgb_active_) {
      rgb_timer_ = this->create_wall_timer(
        rgb_period, std::bind(&VCCSimROS2Node::rgb_timer_callback, this));
    }
    
    if (depth_active_) {
      depth_timer_ = this->create_wall_timer(
        depth_period, std::bind(&VCCSimROS2Node::depth_timer_callback, this));
    }
    
    if (lidar_active_) {
      lidar_timer_ = this->create_wall_timer(
        lidar_period, std::bind(&VCCSimROS2Node::lidar_timer_callback, this));
    }
    
    if (drone_pose_active_) {
      drone_pose_timer_ = this->create_wall_timer(
        drone_pose_period, std::bind(&VCCSimROS2Node::drone_pose_timer_callback, this));
    }
    
    // Create TF timer (always active)
    tf_timer_ = this->create_wall_timer(
      tf_period, std::bind(&VCCSimROS2Node::tf_timer_callback, this));
    
    // Publish static transforms
    publish_static_transforms();
    
    RCLCPP_INFO(this->get_logger(), "VCCSimROS2Node initialized. Connecting to %s:%d", 
                host.c_str(), port);
    
    // Log component activation status
    RCLCPP_INFO(this->get_logger(), "RGB Camera: %s (%.1f Hz)", 
                rgb_active_ ? "ACTIVE" : "INACTIVE", rgb_freq);
    RCLCPP_INFO(this->get_logger(), "Depth Camera: %s (%.1f Hz)", 
                depth_active_ ? "ACTIVE" : "INACTIVE", depth_freq);
    RCLCPP_INFO(this->get_logger(), "LiDAR: %s (%.1f Hz)", 
                lidar_active_ ? "ACTIVE" : "INACTIVE", lidar_freq);
    RCLCPP_INFO(this->get_logger(), "Drone Pose: %s (%.1f Hz)", 
                drone_pose_active_ ? "ACTIVE" : "INACTIVE", drone_pose_freq);
    RCLCPP_INFO(this->get_logger(), "TF Publishing: ACTIVE (%.1f Hz)", tf_freq);
  }

  ~VCCSimROS2Node() {
    // Close the client connection
    if (client_) {
      client_->Close();
    }
  }

private:
  // Timer callbacks for each component
  void rgb_timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    try {
      publish_rgb_image(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error fetching RGB image: %s", e.what());
    }
  }
  
  void depth_timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    try {
      publish_depth_image(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error fetching depth image: %s", e.what());
    }
  }
  
  void lidar_timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    try {
      publish_lidar_data(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error fetching LiDAR data: %s", e.what());
    }
  }
  
  void drone_pose_timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    try {
      publish_drone_pose(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error fetching drone pose: %s", e.what());
    }
  }

  void publish_rgb_image(const std::string& robot_name)
  {
    try {
        // Try to get dimensions first
        auto dimensions = client_->GetRGBCameraImageSize(robot_name, rgb_camera_index_);
        int width = std::get<0>(dimensions);
        int height = std::get<1>(dimensions);
        
        // Update our cached dimensions if valid
        if (width > 0 && height > 0) {
            rgb_width_ = width;
            rgb_height_ = height;
        }
        
        // Get the raw image data
        std::vector<uint8_t> rgb_data = client_->GetRGBImageData(robot_name, rgb_camera_index_, VCCTypes::RAW);
        
        // Create ROS image message
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        
        // Set header
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = robot_name + "_rgb_camera";
        
        // Set image properties
        if (rgb_width_ > 0 && rgb_height_ > 0 && !rgb_data.empty()) {
            // Make sure the data size matches the expected size
            size_t expected_size = rgb_width_ * rgb_height_ * 3;  // 3 bytes per pixel for RGB
            
            if (rgb_data.size() != expected_size) {
                RCLCPP_WARN(this->get_logger(), 
                    "RGB image data size mismatch: got %zu bytes, expected %zu bytes. Resizing.",
                    rgb_data.size(), expected_size);
                    
                // In case data is larger than expected, truncate
                if (rgb_data.size() > expected_size) {
                    rgb_data.resize(expected_size);
                }
                // In case data is smaller than expected, pad with zeros
                else if (rgb_data.size() < expected_size) {
                    rgb_data.resize(expected_size, 0);
                }
            }
            
            image_msg->height = rgb_height_;
            image_msg->width = rgb_width_;
            image_msg->encoding = "rgb8";
            image_msg->is_bigendian = false;
            image_msg->step = rgb_width_ * 3;  // 3 bytes per pixel for RGB
            
            // Copy image data
            image_msg->data = rgb_data;  // Direct assignment is more efficient
            
            // Publish the image
            rgb_pub_->publish(std::move(image_msg));
            
            // Also publish camera info
            publish_camera_info(rgb_info_pub_, robot_name + "_rgb_camera", rgb_width_, rgb_height_);
            
            RCLCPP_DEBUG(this->get_logger(), "Published RGB image: %dx%d", rgb_width_, rgb_height_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid RGB image dimensions or empty data");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
    }
  }
  
  void publish_camera_info(
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& publisher,
    const std::string& frame_id,
    int width,
    int height)
  {
    auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>();
    
    // Set header
    info_msg->header.stamp = this->now();
    info_msg->header.frame_id = frame_id;
    
    // Set dimensions
    info_msg->width = width;
    info_msg->height = height;
    
    // Set reasonable default intrinsics (adjust these based on your camera model)
    double fx = width * 0.8;  // Focal length x
    double fy = height * 0.8; // Focal length y
    double cx = width / 2.0;  // Principal point x
    double cy = height / 2.0; // Principal point y
    
    // Set camera matrix (K) - 3x3 intrinsic matrix in row-major format
    info_msg->k = {
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    };
    
    // Set projection matrix (P) - 3x4 projection matrix in row-major format
    info_msg->p = {
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    // Set distortion model and parameters (assume no distortion for simplicity)
    info_msg->distortion_model = "plumb_bob";
    info_msg->d = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Publish camera info
    publisher->publish(std::move(info_msg));
  }

  void publish_depth_image(const std::string& robot_name) {
    try {
      // Try to get depth image dimensions first
      try {
        auto dimensions = client_->GetDepthCameraImageSize(robot_name);
        depth_width_ = std::get<0>(dimensions);
        depth_height_ = std::get<1>(dimensions);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Could not get depth image dimensions: %s", e.what());
        // Use default dimensions as fallback if direct method fails
        if (depth_width_ <= 0 || depth_height_ <= 0) {
          // Default to standard VGA resolution
          depth_width_ = 640;
          depth_height_ = 480;
          
          RCLCPP_INFO(this->get_logger(), "Using default depth image dimensions: %dx%d",
                    depth_width_, depth_height_);
        }
      }
      
      // Get depth camera image data
      auto depth_data = client_->GetDepthCameraImageData(robot_name);
      
      if (depth_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty depth image data");
        return;
      }
      
      // Create a ROS Image message
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      
      // Set header
      image_msg->header.stamp = this->now();
      image_msg->header.frame_id = robot_name + "_depth_camera";
      
      // Check if data size matches the expected size
      size_t expected_size = depth_width_ * depth_height_;
      if (depth_data.size() != expected_size) {
        RCLCPP_WARN(this->get_logger(), 
                  "Depth image data size mismatch: got %zu values, expected %zu. Adjusting dimensions.",
                  depth_data.size(), expected_size);
        
        // Try to adjust dimensions to match data size
        // Maintain aspect ratio as much as possible
        double scale = std::sqrt(static_cast<double>(depth_data.size()) / static_cast<double>(expected_size));
        depth_width_ = static_cast<int>(depth_width_ * scale);
        depth_height_ = static_cast<int>(depth_height_ * scale);
        expected_size = depth_width_ * depth_height_;
        
        RCLCPP_INFO(this->get_logger(), "Adjusted depth dimensions to %dx%d (target size: %zu)",
                  depth_width_, depth_height_, expected_size);
        
        // If still not matching, add padding or truncate
        if (depth_data.size() > expected_size) {
          depth_data.resize(expected_size);
          RCLCPP_WARN(this->get_logger(), "Truncated depth data to fit dimensions");
        } else if (depth_data.size() < expected_size) {
          depth_data.resize(expected_size, 0.0f);
          RCLCPP_WARN(this->get_logger(), "Padded depth data to fit dimensions");
        }
      }
      
      image_msg->height = depth_height_;
      image_msg->width = depth_width_;
      
      // Using 32FC1 for floating point depth values - adjust if your format differs
      image_msg->encoding = "32FC1";
      image_msg->is_bigendian = false;
      image_msg->step = depth_width_ * sizeof(float);
      
      // Copy depth data
      const size_t data_size = depth_data.size() * sizeof(float);
      image_msg->data.resize(data_size);
      std::memcpy(image_msg->data.data(), depth_data.data(), data_size);
      
      // Publish the depth image
      depth_pub_->publish(std::move(image_msg));
      
      // Also publish camera info
      publish_camera_info(depth_info_pub_, robot_name + "_depth_camera", depth_width_, depth_height_);
      
      RCLCPP_DEBUG(this->get_logger(), "Published depth image: %dx%d with %zu values", 
                  depth_width_, depth_height_, depth_data.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing depth image: %s", e.what());
    }
  }
  
  void publish_lidar_data(const std::string& robot_name) {
    try {
      // Get LiDAR points and odometry
      auto [lidar_points, lidar_odom] = client_->GetLidarDataAndOdom(robot_name);
      
      if (lidar_points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty LiDAR point cloud");
        return;
      }
      
      // Create a PointCloud2 message
      auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
      
      // Set header - since points are in world coordinates, frame should be "map"
      cloud_msg->header.stamp = this->now();
      cloud_msg->header.frame_id = "map";  // Use map instead of robot frame
      
      // Set point cloud properties
      cloud_msg->height = 1;
      cloud_msg->width = lidar_points.size();
      
      // Define point cloud fields (x, y, z)
      cloud_msg->fields.resize(3);
      cloud_msg->fields[0].name = "x";
      cloud_msg->fields[0].offset = 0;
      cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg->fields[0].count = 1;
      
      cloud_msg->fields[1].name = "y";
      cloud_msg->fields[1].offset = 4;
      cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg->fields[1].count = 1;
      
      cloud_msg->fields[2].name = "z";
      cloud_msg->fields[2].offset = 8;
      cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg->fields[2].count = 1;
      
      cloud_msg->point_step = 12; // 3 floats (x,y,z) * 4 bytes
      cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
      cloud_msg->is_dense = true;
      
      // Use a more efficient approach to populate the point cloud
      cloud_msg->data.resize(lidar_points.size() * cloud_msg->point_step);
      
      // Use sensor_msgs point cloud iterator for safer data filling
      sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
      
      for (const auto& point : lidar_points) {
        // Transform point from left-handed (UE) to right-handed (ROS) and cm to m
        // Since these are world coordinates, we only need to transform the coordinate system
        // and convert units, not apply drone rotation
        *iter_x = point.x / 100.0f;    // cm to m
        *iter_y = -point.y / 100.0f;   // negate Y and cm to m (left to right-handed)
        *iter_z = point.z / 100.0f;    // cm to m
        
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      
      // Publish the point cloud
      lidar_pub_->publish(std::move(cloud_msg));
      
      RCLCPP_DEBUG(this->get_logger(), "Published LiDAR point cloud with %zu points in world coordinates", 
                  lidar_points.size());
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing LiDAR data: %s", e.what());
    }
  }

  void publish_drone_pose(const std::string& robot_name) {
    try {
      // Get drone pose
      VCCTypes::Pose drone_pose = client_->GetDronePose(robot_name);
      
      // Create a PoseStamped message
      auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
      
      // Set header
      pose_msg->header.stamp = this->now();
      pose_msg->header.frame_id = "map"; // Map is our fixed reference frame
      
      // Set position - transform from UE to ROS coordinates and cm to m
      pose_msg->pose.position = transform_position(drone_pose);
      
      // Convert rotation - transform from UE to ROS orientation
      tf2::Quaternion q = transform_rotation(drone_pose);
      
      pose_msg->pose.orientation.w = q.w();
      pose_msg->pose.orientation.x = q.x();
      pose_msg->pose.orientation.y = q.y();
      pose_msg->pose.orientation.z = q.z();
      
      // Publish the pose
      drone_pose_pub_->publish(std::move(pose_msg));
      
      RCLCPP_DEBUG(this->get_logger(), "Published drone pose: [%.2f, %.2f, %.2f] [q: w=%.2f x=%.2f y=%.2f z=%.2f] (transformed to ROS coordinates)",
                  pose_msg->pose.position.x, 
                  pose_msg->pose.position.y, 
                  pose_msg->pose.position.z,
                  q.w(), q.x(), q.y(), q.z());
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing drone pose: %s", e.what());
    }
  }
    
  void tf_timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    try {
      publish_transforms(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing transforms: %s", e.what());
    }
  }
  
  void publish_transforms(const std::string& robot_name) {
    // Get the latest drone pose
    VCCTypes::Pose drone_pose;
    try {
      drone_pose = client_->GetDronePose(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error getting drone pose for TF: %s", e.what());
      return;
    }
    
    // Current timestamp for all transforms
    rclcpp::Time now = this->now();
    
    // Transform 1: map -> drone_base
    geometry_msgs::msg::TransformStamped t_map_drone;
    t_map_drone.header.stamp = now;
    t_map_drone.header.frame_id = "map";
    t_map_drone.child_frame_id = robot_name + "_base";
    
    // Set translation - transform from UE to ROS coordinates and cm to m
    auto position = transform_position(drone_pose);
    t_map_drone.transform.translation.x = position.x;
    t_map_drone.transform.translation.y = position.y;
    t_map_drone.transform.translation.z = position.z;
    
    // Set rotation - transform from UE to ROS orientation
    tf2::Quaternion q_drone = transform_rotation(drone_pose);
    
    t_map_drone.transform.rotation.w = q_drone.w();
    t_map_drone.transform.rotation.x = q_drone.x();
    t_map_drone.transform.rotation.y = q_drone.y();
    t_map_drone.transform.rotation.z = q_drone.z();
    
    // Send the transform
    tf_broadcaster_->sendTransform(t_map_drone);
  }

  void publish_static_transforms() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    
    // Vector to hold all static transforms
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    
    // Current timestamp for all transforms
    rclcpp::Time now = this->now();
    
    // Transform: drone_base -> rgb_camera
    geometry_msgs::msg::TransformStamped t_base_rgb;
    t_base_rgb.header.stamp = now;
    t_base_rgb.header.frame_id = robot_name + "_base";
    t_base_rgb.child_frame_id = robot_name + "_rgb_camera";
    
    // Set the RGB camera position (adjust as needed)
    t_base_rgb.transform.translation.x = 0.1;  // 10cm forward
    t_base_rgb.transform.translation.y = 0.0;
    t_base_rgb.transform.translation.z = -0.05; // 5cm down
    
    // Set rotation (pointing forward)
    tf2::Quaternion q_rgb;
    q_rgb.setRPY(0, 0, 0);  // No rotation relative to drone
    q_rgb.normalize();
    
    t_base_rgb.transform.rotation.w = q_rgb.w();
    t_base_rgb.transform.rotation.x = q_rgb.x();
    t_base_rgb.transform.rotation.y = q_rgb.y();
    t_base_rgb.transform.rotation.z = q_rgb.z();
    
    static_transforms.push_back(t_base_rgb);
    
    // Transform: drone_base -> depth_camera
    geometry_msgs::msg::TransformStamped t_base_depth;
    t_base_depth.header.stamp = now;
    t_base_depth.header.frame_id = robot_name + "_base";
    t_base_depth.child_frame_id = robot_name + "_depth_camera";
    
    // Set the depth camera position (adjust as needed)
    t_base_depth.transform.translation.x = 0.1;  // 10cm forward
    t_base_depth.transform.translation.y = 0.0;
    t_base_depth.transform.translation.z = -0.05; // 5cm down, same as RGB camera
    
    // Set rotation (pointing forward)
    tf2::Quaternion q_depth;
    q_depth.setRPY(0, 0, 0);  // No rotation relative to drone
    q_depth.normalize();
    
    t_base_depth.transform.rotation.w = q_depth.w();
    t_base_depth.transform.rotation.x = q_depth.x();
    t_base_depth.transform.rotation.y = q_depth.y();
    t_base_depth.transform.rotation.z = q_depth.z();
    
    static_transforms.push_back(t_base_depth);
    
    // Transform: drone_base -> lidar
    geometry_msgs::msg::TransformStamped t_base_lidar;
    t_base_lidar.header.stamp = now;
    t_base_lidar.header.frame_id = robot_name + "_base";
    t_base_lidar.child_frame_id = robot_name + "_lidar";
    
    // Set the LiDAR position (adjust as needed)
    t_base_lidar.transform.translation.x = 0.0;  // Center
    t_base_lidar.transform.translation.y = 0.0;
    t_base_lidar.transform.translation.z = 0.05; // 5cm up
    
    // Set rotation (standard orientation)
    tf2::Quaternion q_lidar;
    q_lidar.setRPY(0, 0, 0);
    q_lidar.normalize();
    
    t_base_lidar.transform.rotation.w = q_lidar.w();
    t_base_lidar.transform.rotation.x = q_lidar.x();
    t_base_lidar.transform.rotation.y = q_lidar.y();
    t_base_lidar.transform.rotation.z = q_lidar.z();
    
    static_transforms.push_back(t_base_lidar);
    
    // Send all static transforms
    static_tf_broadcaster_->sendTransform(static_transforms);
    
    RCLCPP_INFO(this->get_logger(), "Published static transforms for sensors");
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCCSimROS2Node>());
  rclcpp::shutdown();
  return 0;
}