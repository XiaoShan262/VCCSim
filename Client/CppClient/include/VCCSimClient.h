#pragma once

#include <string>
#include <memory>
#include <vector>
#include <tuple>

// Forward declarations instead of direct includes
namespace grpc {
    class Channel;
}

namespace VCCSim {
    class Pose;
    class twist;
    class Odometry;
    class RGBCameraImageData;
    class RobotName;
    class PoseOnlyYaw;
}

class VCCSimClient {
public:
    /**
     * Initialize the VCCSim client.
     *
     * @param host Server hostname
     * @param port Server port number
     */
    VCCSimClient(const std::string& host = "localhost", int port = 50996);

    /**
     * Destructor that closes the gRPC channel.
     */
    ~VCCSimClient();

    // LiDAR Service Methods
    /**
     * Get LiDAR data for a robot.
     *
     * @param robot_name Name of the robot
     * @return Vector of (x, y, z) tuples representing LiDAR points
     */
    std::vector<std::tuple<float, float, float>> GetLidarData(const std::string& robot_name);

    /**
     * Get LiDAR odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Tuple of pose and twist
     */
    std::tuple<VCCSim::Pose, VCCSim::twist> GetLidarOdom(const std::string& robot_name);

    /**
     * Get both LiDAR data and odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Tuple of points vector and odometry
     */
    std::tuple<std::vector<std::tuple<float, float, float>>, VCCSim::Odometry> GetLidarDataAndOdom(const std::string& robot_name);

    // Depth Camera Service Methods
    /**
     * Get depth camera point data for a robot.
     *
     * @param robot_name Name of the robot
     * @return Vector of (x, y, z) tuples representing depth camera points
     */
    std::vector<std::tuple<float, float, float>> GetDepthCameraPointData(const std::string& robot_name);

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
    VCCSim::Odometry GetDepthCameraOdom(const std::string& robot_name);

    // RGB Camera Service Methods
    /**
     * Get RGB camera odometry for a robot.
     *
     * @param robot_name Name of the robot
     * @return Odometry of the RGB camera
     */
    VCCSim::Odometry GetRGBCameraOdom(const std::string& robot_name);

    /**
     * Get RGB camera image data for a specific camera index.
     *
     * @param robot_name Name of the robot
     * @param index Index of the camera
     * @return RGB camera image data
     */
    VCCSim::RGBCameraImageData GetRGBIndexedCameraImageData(const std::string& robot_name, int index);

    // Drone Service Methods
    /**
     * Get drone pose.
     *
     * @param robot_name Name of the robot
     * @return Pose of the drone
     */
    VCCSim::Pose GetDronePose(const std::string& robot_name);

    /**
     * Send drone pose.
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
     * @param poses Vector of (x, y, z, roll, pitch, yaw) tuples representing poses
     * @return True if successful, false otherwise
     */
    bool SendDronePath(const std::string& name, const std::vector<std::tuple<float, float, float, float, float, float>>& poses);

    // Car Service Methods
    /**
     * Get car odometry.
     *
     * @param robot_name Name of the robot
     * @return Odometry of the car
     */
    VCCSim::Odometry GetCarOdom(const std::string& robot_name);

    /**
     * Send car pose.
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
     * @param poses Vector of (x, y, z, yaw) tuples representing poses
     * @return True if successful, false otherwise
     */
    bool SendCarPath(const std::string& name, const std::vector<std::tuple<float, float, float, float>>& poses);

    // Flash Service Methods
    /**
     * Get flash pose.
     *
     * @param robot_name Name of the robot
     * @return Pose of the flash
     */
    VCCSim::Pose GetFlashPose(const std::string& robot_name);

    /**
     * Send flash pose.
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
     * @param poses Vector of (x, y, z, roll, pitch, yaw) tuples representing poses
     * @return True if successful, false otherwise
     */
    bool SendFlashPath(const std::string& name, const std::vector<std::tuple<float, float, float, float, float, float>>& poses);

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

    // Mesh Service Methods
    /**
     * Send mesh data.
     *
     * @param data Mesh data bytes
     * @param format Format of the mesh
     * @param version Version of the mesh
     * @param simplified Whether the mesh is simplified
     * @param transform_pose Transformation pose as (x, y, z, roll, pitch, yaw)
     * @return True if successful, false otherwise
     */
    bool SendMesh(const std::string& data, int format, int version, bool simplified,
                 const std::tuple<float, float, float, float, float, float>& transform_pose);

    /**
     * Send global mesh data.
     *
     * @param data Mesh data bytes
     * @param format Format of the mesh
     * @param version Version of the mesh
     * @param simplified Whether the mesh is simplified
     * @param transform_pose Transformation pose as (x, y, z, roll, pitch, yaw)
     * @return ID of the created mesh
     */
    int SendGlobalMesh(const std::string& data, int format, int version, bool simplified,
                     const std::tuple<float, float, float, float, float, float>& transform_pose);

    /**
     * Remove a global mesh by its ID.
     *
     * @param mesh_id ID of the mesh to remove
     * @return True if successful, false otherwise
     */
    bool RemoveGlobalMesh(int mesh_id);

    // Point Cloud Service Methods
    /**
     * Send colored point cloud data.
     *
     * @param points Vector of (x, y, z) tuples representing points
     * @param colors Vector of color values for each point
     * @return True if successful, false otherwise
     */
    bool SendPointCloudWithColor(const std::vector<std::tuple<float, float, float>>& points,
                               const std::vector<int>& colors);

private:
    // Helper methods
    VCCSim::RobotName CreateRobotName(const std::string& name);
    VCCSim::Pose CreatePose(float x, float y, float z, float roll, float pitch, float yaw);
    VCCSim::PoseOnlyYaw CreatePoseOnlyYaw(float x, float y, float z, float yaw);

    // Private implementation class to hide the details
    class Impl;
    std::unique_ptr<Impl> pImpl;
};