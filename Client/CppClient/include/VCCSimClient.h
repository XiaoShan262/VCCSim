#ifndef VCCSIM_CLIENT_H
#define VCCSIM_CLIENT_H

#include <string>
#include <memory>
#include <vector>
#include <tuple>
#include <functional>
#include <cstdint>

// Forward declarations instead of direct includes
namespace grpc {
    class Channel;
}

// Forward declare the Protocol Buffer Format enum
namespace VCCSim {
    enum Format : int;
}

// Simple structs to replace Protocol Buffer generated classes in a separate namespace
namespace VCCTypes {
    // Image formats enum (matches Protocol Buffer enum values)
    enum Format {
        PNG = 0,
        JPEG = 1,
        RAW = 2
    };
    
    // Simple position struct
    struct Position {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };
    
    // Simple pose struct with full rotation
    struct Pose {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
    };

    // Simple pose struct with only yaw rotation
    struct PoseYaw {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float yaw = 0.0f;
    };

    // Simple twist struct for velocity
    struct Twist {
        float linear_x = 0.0f;
        float linear_y = 0.0f;
        float linear_z = 0.0f;
        float angular_x = 0.0f;
        float angular_y = 0.0f;
        float angular_z = 0.0f;
    };

    // Simple odometry struct
    struct Odometry {
        Pose pose;
        Twist twist;
    };

    // Simple RGB image data struct
    struct RGBImage {
        uint32_t width = 0;
        uint32_t height = 0;
        std::vector<uint8_t> data;
        Format format = PNG;
        uint32_t timestamp = 0;
    };
    
    // Simple 3D point struct
    struct Point {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };
}

class VCCSimClient {
public:
    /**
     * Initialize the VCCSim client.
     *
     * @param host Server hostname
     * @param port Server port number
     * @param max_message_length Maximum message length in bytes (default: 20MB)
     */
    VCCSimClient(const std::string& host = "localhost", int port = 50996, 
                int max_message_length = 20 * 1024 * 1024);

    /**
     * Destructor that closes the gRPC channel.
     */
    ~VCCSimClient();
    
    /**
     * Close the gRPC channel explicitly.
     */
    void Close();

    // RGB Camera Methods
    /**
     * Get RGB camera raw image data.
     *
     * @param robot_name Name of the robot
     * @param index Index of the camera
     * @param format Image format (PNG, JPEG, or RAW), default is PNG
     * @return Vector of bytes containing the image data
     */
    std::vector<uint8_t> GetRGBImageData(const std::string& robot_name, int index, 
                                        VCCTypes::Format format = VCCTypes::PNG);

    /**
     * Get RGB camera image data and save directly to a file.
     *
     * @param robot_name Name of the robot
     * @param index Index of the camera
     * @param output_path Path to save the image
     * @param format Image format (PNG, JPEG, or RAW), default is PNG
     * @return True if saving was successful, false otherwise
     */
    bool GetAndSaveRGBImage(const std::string& robot_name, int index, 
                           const std::string& output_path, VCCTypes::Format format = VCCTypes::PNG);

    /**
     * Get RGB camera image data for a specific camera index.
     *
     * @param robot_name Name of the robot
     * @param index Index of the camera
     * @param format Image format (PNG, JPEG, or RAW), default is PNG
     * @return RGB image data
     */
    VCCTypes::RGBImage GetRGBIndexedCameraImageData(const std::string& robot_name, int index, 
                                                VCCTypes::Format format = VCCTypes::PNG);
                                                
    /**
     * Save RGB image data to a file.
     *
     * @param image_data RGB image data
     * @param output_path Path to save the image
     * @return True if saving was successful, false otherwise
     */
    bool SaveRGBImage(const VCCTypes::RGBImage& image_data, const std::string& output_path);
    
    /**
     * Get RGB camera odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Odometry of the RGB camera
     */
    
    VCCTypes::Odometry GetRGBCameraOdom(const std::string& robot_name);
        /**
     * Get RGB camera image size directly without fetching image data.
     *
     * @param robot_name Name of the robot
     * @param index Index of the camera
     * @return Tuple of (width, height)
     */
    std::tuple<int, int> GetRGBCameraImageSize(const std::string& robot_name, int index);

    /**
     * Get depth camera image size directly without fetching image data.
     *
     * @param robot_name Name of the robot
     * @return Tuple of (width, height)
     */
    std::tuple<int, int> GetDepthCameraImageSize(const std::string& robot_name);

    // LiDAR Methods
    /**
     * Get LiDAR data for a robot.
     *
     * @param robot_name Name of the robot
     * @return Vector of points representing LiDAR points
     */
    std::vector<VCCTypes::Point> GetLidarData(const std::string& robot_name);

    /**
     * Get LiDAR odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Odometry containing pose and twist
     */
    VCCTypes::Odometry GetLidarOdom(const std::string& robot_name);

    /**
     * Get both LiDAR data and odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Tuple of points vector and odometry
     */
    std::tuple<std::vector<VCCTypes::Point>, VCCTypes::Odometry> GetLidarDataAndOdom(const std::string& robot_name);

    // Depth Camera Methods
    /**
     * Get depth camera point data for a robot.
     *
     * @param robot_name Name of the robot
     * @return Vector of points representing depth camera points
     */
    std::vector<VCCTypes::Point> GetDepthCameraPointData(const std::string& robot_name);

    /**
     * Get depth camera image data for a robot.
     *
     * @param robot_name Name of the robot
     * @return Vector of float values representing the depth image
     */
    std::vector<float> GetDepthCameraImageData(const std::string& robot_name);

    /**
     * Get depth camera odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Odometry of the depth camera
     */
    VCCTypes::Odometry GetDepthCameraOdom(const std::string& robot_name);

    // Drone Methods
    /**
     * Get drone pose.
     *
     * @param robot_name Name of the robot
     * @return Pose of the drone
     */
    VCCTypes::Pose GetDronePose(const std::string& robot_name);

    /**
     * Send drone pose.
     *
     * @param name Name of the drone
     * @param pose Pose to send
     * @return True if successful, false otherwise
     */
    bool SendDronePose(const std::string& name, const VCCTypes::Pose& pose);

    /**
     * Send drone pose with individual components.
     *
     * @param name Name of the drone
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param z Z-coordinate
     * @param roll Roll angle
     * @param pitch Pitch angle
     * @param yaw Yaw angle
     * @return True if successful, false otherwise
     */
    bool SendDronePose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw);

    /**
     * Send drone path as a list of poses.
     *
     * @param name Name of the drone
     * @param poses Vector of poses
     * @return True if successful, false otherwise
     */
    bool SendDronePath(const std::string& name, const std::vector<VCCTypes::Pose>& poses);

    // Car Methods
    /**
     * Get car odometry.
     *
     * @param robot_name Name of the robot
     * @return Odometry of the car
     */
    VCCTypes::Odometry GetCarOdom(const std::string& robot_name);

    /**
     * Send car pose.
     *
     * @param name Name of the car
     * @param pose Pose to send (only yaw angle is used)
     * @return True if successful, false otherwise
     */
    bool SendCarPose(const std::string& name, const VCCTypes::PoseYaw& pose);

    /**
     * Send car pose with individual components.
     *
     * @param name Name of the car
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param z Z-coordinate
     * @param yaw Yaw angle
     * @return True if successful, false otherwise
     */
    bool SendCarPose(const std::string& name, float x, float y, float z, float yaw);

    /**
     * Send car path as a list of poses.
     *
     * @param name Name of the car
     * @param poses Vector of poses
     * @return True if successful, false otherwise
     */
    bool SendCarPath(const std::string& name, const std::vector<VCCTypes::PoseYaw>& poses);

    // Flash Methods
    /**
     * Get flash pose.
     *
     * @param robot_name Name of the robot
     * @return Pose of the flash
     */
    VCCTypes::Pose GetFlashPose(const std::string& robot_name);

    /**
     * Send flash pose.
     *
     * @param name Name of the flash
     * @param pose Pose to send
     * @return True if successful, false otherwise
     */
    bool SendFlashPose(const std::string& name, const VCCTypes::Pose& pose);

    /**
     * Send flash pose with individual components.
     *
     * @param name Name of the flash
     * @param x X-coordinate
     * @param y Y-coordinate
     * @param z Z-coordinate
     * @param roll Roll angle
     * @param pitch Pitch angle
     * @param yaw Yaw angle
     * @return True if successful, false otherwise
     */
    bool SendFlashPose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw);

    /**
     * Send flash path as a list of poses.
     *
     * @param name Name of the flash
     * @param poses Vector of poses
     * @return True if successful, false otherwise
     */
    bool SendFlashPath(const std::string& name, const std::vector<VCCTypes::Pose>& poses);

    /**
     * Check if flash is ready.
     *
     * @param robot_name Name of the robot
     * @return True if ready, false otherwise
     */
    bool CheckFlashReady(const std::string& robot_name);

    /**
     * Move flash to next position.
     *
     * @param robot_name Name of the robot
     * @return True if successful, false otherwise
     */
    bool MoveToNext(const std::string& robot_name);

    // Mesh Methods
    /**
     * Send mesh data.
     *
     * @param data Mesh data bytes
     * @param format Format of the mesh
     * @param version Version of the mesh
     * @param simplified Whether the mesh is simplified
     * @param transform_pose Transformation pose
     * @return True if successful, false otherwise
     */
    bool SendMesh(const std::string& data, int format, int version, bool simplified,
                 const VCCTypes::Pose& transform_pose);

    /**
     * Send global mesh data.
     *
     * @param data Mesh data bytes
     * @param format Format of the mesh
     * @param version Version of the mesh
     * @param simplified Whether the mesh is simplified
     * @param transform_pose Transformation pose
     * @return ID of the created mesh
     */
    int SendGlobalMesh(const std::string& data, int format, int version, bool simplified,
                     const VCCTypes::Pose& transform_pose);

    /**
     * Remove a global mesh by its ID.
     *
     * @param mesh_id ID of the mesh to remove
     * @return True if successful, false otherwise
     */
    bool RemoveGlobalMesh(int mesh_id);

    // Point Cloud Methods
    /**
     * Send colored point cloud data.
     *
     * @param points Vector of points
     * @param colors Vector of color values for each point
     * @return True if successful, false otherwise
     */
    bool SendPointCloudWithColor(const std::vector<VCCTypes::Point>& points,
                               const std::vector<int>& colors);

private:
    // Private implementation class to hide the details
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

#endif // VCCSIM_CLIENT_H