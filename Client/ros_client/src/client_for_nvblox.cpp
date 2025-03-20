#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "VCCSimClient.h"

using namespace std::chrono_literals;

class VCCSimROS2Node : public rclcpp::Node {
private:
  // VCCSimClient instance
  std::unique_ptr<VCCSimClient> client_;
  
  // Publishers for RGB and depth images
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  
  // Timer for periodic image fetching
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Camera index
  int rgb_camera_index_;

public:
  VCCSimROS2Node() : Node("vccsim_ros2_node") {
    // Declare and get parameters
    this->declare_parameter<std::string>("vccsim_host", "172.31.178.18");
    this->declare_parameter<int>("vccsim_port", 50996);
    this->declare_parameter<std::string>("robot_name", "Mavic");
    this->declare_parameter<int>("rgb_camera_index", 0);
    
    std::string host = this->get_parameter("vccsim_host").as_string();
    int port = this->get_parameter("vccsim_port").as_int();
    rgb_camera_index_ = this->get_parameter("rgb_camera_index").as_int();
    
    // Initialize the VCCSimClient
    client_ = std::make_unique<VCCSimClient>(host, port);
    
    // Create publishers
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "vccsim/rgb_image", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "vccsim/depth_image", 10);
    
    // Create timer (5Hz = 200ms)
    timer_ = this->create_wall_timer(
      200ms, std::bind(&VCCSimROS2Node::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "VCCSimROS2Node initialized. Connecting to %s:%d", 
                host.c_str(), port);
  }

  ~VCCSimROS2Node() {
    // Close the client connection
    if (client_) {
      client_->Close();
    }
  }

private:
  void timer_callback() {
    std::string robot_name = this->get_parameter("robot_name").as_string();
    
    try {
      // Get RGB image
      publish_rgb_image(robot_name);
      
      // Get depth image
      publish_depth_image(robot_name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error fetching images: %s", e.what());
    }
  }

  void publish_rgb_image(const std::string& robot_name)
  {
    try {
        // Use the new methods that don't require direct access to protobuf types
        auto dimensions = client_->GetRGBImageDimensions(robot_name, rgb_camera_index_);
        int width = std::get<0>(dimensions);
        int height = std::get<1>(dimensions);
        
        // Get the raw image data
        std::vector<uint8_t> rgb_data = client_->GetRGBImageData(robot_name, rgb_camera_index_);
        
        // Create ROS image message
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        
        // Set header
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = robot_name + "_rgb_camera";
        
        // Set image properties
        if (width > 0 && height > 0 && !rgb_data.empty()) {
            image_msg->height = height;
            image_msg->width = width;
            image_msg->encoding = "rgb8";
            image_msg->is_bigendian = false;
            image_msg->step = width * 3;  // 3 bytes per pixel for RGB
            
            // Copy image data
            image_msg->data.resize(rgb_data.size());
            std::memcpy(image_msg->data.data(), rgb_data.data(), rgb_data.size());
            
            // Publish the image
            rgb_pub_->publish(std::move(image_msg));
            
            RCLCPP_DEBUG(this->get_logger(), "Published RGB image: %dx%d", width, height);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid RGB image dimensions");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
    }
  }

  void publish_depth_image(const std::string& robot_name) {
    try {
      // Get depth camera image data
      auto depth_data = client_->GetDepthCameraImageData(robot_name);
      
      // Create a ROS Image message
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      
      // Set header
      image_msg->header.stamp = this->now();
      image_msg->header.frame_id = robot_name + "_depth_camera";
      
      // Depth images typically have fixed dimensions
      // You may need to adjust these or get them from the client API
      const int depth_width = 640;  // Adjust based on your actual depth camera resolution
      const int depth_height = 480; // Adjust based on your actual depth camera resolution
      
      if (!depth_data.empty()) {
        image_msg->height = depth_height;
        image_msg->width = depth_width;
        
        // Using 32FC1 for floating point depth values - adjust if your format differs
        image_msg->encoding = "32FC1";
        image_msg->is_bigendian = false;
        image_msg->step = depth_width * sizeof(float);
        
        // Copy depth data
        const size_t data_size = depth_data.size() * sizeof(float);
        image_msg->data.resize(data_size);
        std::memcpy(image_msg->data.data(), depth_data.data(), data_size);
        
        // Publish the depth image
        depth_pub_->publish(std::move(image_msg));
        
        RCLCPP_DEBUG(this->get_logger(), "Published depth image: %dx%d with %zu values", 
                    depth_width, depth_height, depth_data.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "Received empty depth image data");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing depth image: %s", e.what());
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VCCSimROS2Node>());
  rclcpp::shutdown();
  return 0;
}