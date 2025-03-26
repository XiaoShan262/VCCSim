// vccsim_node_mt.hpp
#ifndef VCCSIM_NODE_MT_HPP
#define VCCSIM_NODE_MT_HPP

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "VCCSimClient.h"

using namespace std::chrono_literals;

// Base sensor node class
class SensorNodeBase : public rclcpp::Node
{
public:
    using SharedPtr = std::shared_ptr<SensorNodeBase>;
    
    SensorNodeBase(const std::string& name, std::shared_ptr<VCCSimClient> client, 
                  const std::string& robot_name, double frequency);
    virtual ~SensorNodeBase();
    
    void start();
    void stop();
    
protected:
    std::shared_ptr<VCCSimClient> client_;
    std::string robot_name_;
    double frequency_;
    bool is_running_;
    std::thread executor_thread_;
    std::string sensor_type_;
    bool enable_timing_;
    
    // Frequency stats
    struct FrequencyStats {
        std::chrono::steady_clock::time_point first_publish_time;
        std::chrono::steady_clock::time_point last_publish_time;
        int publish_count = 0;
        double current_frequency = 0.0;
        std::mutex stats_mutex;
    };
    
    FrequencyStats frequency_stats_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr freq_pub_;
    rclcpp::TimerBase::SharedPtr frequency_monitor_timer_;
    
    // Methods
    virtual void publish_data() = 0;
    void update_frequency_stats();
    void monitor_frequency();
};

// RGB Camera Node
class RGBCameraNode : public SensorNodeBase
{
public:
    RGBCameraNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name, 
                 int camera_index, double frequency);
    
protected:
    void publish_data() override;
    
private:
    int camera_index_;
    int width_;
    int height_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::Image convert_to_ros_image(const VCCSim::RGBCameraImageData& vcc_image);
};

// Depth Camera Node
class DepthCameraNode : public SensorNodeBase
{
public:
    DepthCameraNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name, 
                   double frequency);
    
protected:
    void publish_data() override;
    
private:
    int width_;
    int height_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::Image convert_to_ros_depth_image(const std::vector<float>& depth_data, 
                                                     int width, int height);
};

// LiDAR Node
class LidarNode : public SensorNodeBase
{
public:
    LidarNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name, 
             double frequency);
    
protected:
    void publish_data() override;
    
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    sensor_msgs::msg::PointCloud2 convert_to_ros_pointcloud(const std::vector<VCCSim::Point>& vcc_points);
};

// Drone Pose Node
class DronePoseNode : public SensorNodeBase
{
public:
    DronePoseNode(std::shared_ptr<VCCSimClient> client, const std::string& robot_name, 
                 double frequency);
    
protected:
    void publish_data() override;
    
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseStamped convert_to_ros_pose(const VCCSim::Pose& vcc_pose);
};

// Node Manager
class VCCSimNodeManager
{
public:
    VCCSimNodeManager();
    ~VCCSimNodeManager();
    
    void init(int argc, char* argv[]);
    void start();
    void stop();
    
private:
    std::shared_ptr<VCCSimClient> client_;
    std::string robot_name_;
    int rgb_camera_index_;
    
    // Configuration flags and parameters
    bool rgb_active_;
    bool depth_active_;
    bool lidar_active_;
    bool drone_pose_active_;
    
    double rgb_frequency_;
    double depth_frequency_;
    double lidar_frequency_;
    double drone_pose_frequency_;
    
    // Nodes
    std::shared_ptr<RGBCameraNode> rgb_node_;
    std::shared_ptr<DepthCameraNode> depth_node_;
    std::shared_ptr<LidarNode> lidar_node_;
    std::shared_ptr<DronePoseNode> drone_node_;
    std::shared_ptr<rclcpp::Node> param_node_;
    
    // Thread control
    std::atomic<bool> is_running_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    void publish_transforms();
};

#endif // VCCSIM_NODE_MT_HPP