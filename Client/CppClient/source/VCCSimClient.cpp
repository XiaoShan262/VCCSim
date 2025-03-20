#include "VCCSimClient.h"

// Include the actual gRPC and protobuf headers only in the implementation file
#include "VCCSim.pb.h"
#include "VCCSim.grpc.pb.h"

// Use the Format enum from the protobuf file for clarity in the implementation
using Format = VCCSim::Format;
#include <grpcpp/grpcpp.h>
#include <fstream>
#include <stdexcept>

// RGBImageUtils implementation
bool RGBImageUtils::SaveRGBImage(const VCCSim::RGBCameraImageData& image_data, const std::string& output_path) {
    try {
        // For PNG, JPEG, and RAW formats, we can write directly to file
        std::ofstream file(output_path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        file.write(image_data.data().c_str(), image_data.data().size());
        file.close();
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

std::vector<uint8_t> RGBImageUtils::ProcessRGBImageData(const VCCSim::RGBCameraImageData& image_data) {
    // Convert the image data string to a vector of bytes
    std::vector<uint8_t> result(image_data.data().begin(), image_data.data().end());
    return result;
}

std::tuple<int, int> RGBImageUtils::GetImageDimensions(const VCCSim::RGBCameraImageData& image_data) {
    return std::make_tuple(image_data.width(), image_data.height());
}

// Private implementation class (PIMPL idiom)
class VCCSimClient::Impl {
public:
    Impl(const std::string& host, int port, int max_message_length) {
        // Configure channel options for larger messages
        grpc::ChannelArguments args;
        args.SetMaxSendMessageSize(max_message_length);
        args.SetMaxReceiveMessageSize(max_message_length);
        
        // Create the gRPC channel with options
        std::string server_address = host + ":" + std::to_string(port);
        channel_ = grpc::CreateCustomChannel(server_address, grpc::InsecureChannelCredentials(), args);
        
        // Initialize the service stubs
        lidar_service_ = std::unique_ptr<VCCSim::LidarService::Stub>(VCCSim::LidarService::NewStub(channel_).release());
        depth_camera_service_ = std::unique_ptr<VCCSim::DepthCameraService::Stub>(VCCSim::DepthCameraService::NewStub(channel_).release());
        rgb_camera_service_ = std::unique_ptr<VCCSim::RGBCameraService::Stub>(VCCSim::RGBCameraService::NewStub(channel_).release());
        drone_service_ = std::unique_ptr<VCCSim::DroneService::Stub>(VCCSim::DroneService::NewStub(channel_).release());
        car_service_ = std::unique_ptr<VCCSim::CarService::Stub>(VCCSim::CarService::NewStub(channel_).release());
        flash_service_ = std::unique_ptr<VCCSim::FlashService::Stub>(VCCSim::FlashService::NewStub(channel_).release());
        mesh_service_ = std::unique_ptr<VCCSim::MeshService::Stub>(VCCSim::MeshService::NewStub(channel_).release());
        point_cloud_service_ = std::unique_ptr<VCCSim::PointCloudService::Stub>(VCCSim::PointCloudService::NewStub(channel_).release());
    }
    
    ~Impl() {
        // Close channel if needed
        if (channel_) {
            channel_.reset();
        }
    }
    
    void Close() {
        // Explicitly shut down the channel
        if (channel_) {
            channel_.reset();
        }
    }
    
    // Helper methods
    VCCSim::RobotName CreateRobotName(const std::string& name) {
        VCCSim::RobotName robot_name;
        robot_name.set_name(name);
        return robot_name;
    }
    
    VCCSim::Pose CreatePose(float x, float y, float z, float roll, float pitch, float yaw) {
        VCCSim::Pose pose;
        pose.set_x(x);
        pose.set_y(y);
        pose.set_z(z);
        pose.set_roll(roll);
        pose.set_pitch(pitch);
        pose.set_yaw(yaw);
        return pose;
    }
    
    VCCSim::PoseOnlyYaw CreatePoseOnlyYaw(float x, float y, float z, float yaw) {
        VCCSim::PoseOnlyYaw pose;
        pose.set_x(x);
        pose.set_y(y);
        pose.set_z(z);
        pose.set_yaw(yaw);
        return pose;
    }
    
    // gRPC channel and stubs
    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<VCCSim::LidarService::Stub> lidar_service_;
    std::unique_ptr<VCCSim::DepthCameraService::Stub> depth_camera_service_;
    std::unique_ptr<VCCSim::RGBCameraService::Stub> rgb_camera_service_;
    std::unique_ptr<VCCSim::DroneService::Stub> drone_service_;
    std::unique_ptr<VCCSim::CarService::Stub> car_service_;
    std::unique_ptr<VCCSim::FlashService::Stub> flash_service_;
    std::unique_ptr<VCCSim::MeshService::Stub> mesh_service_;
    std::unique_ptr<VCCSim::PointCloudService::Stub> point_cloud_service_;
};

// Constructor
VCCSimClient::VCCSimClient(const std::string& host, int port, int max_message_length)
    : pImpl(std::make_unique<Impl>(host, port, max_message_length)) {
}

// Destructor
VCCSimClient::~VCCSimClient() = default;

// Close method
void VCCSimClient::Close() {
    pImpl->Close();
}

std::tuple<int, int> VCCSimClient::GetRGBImageDimensions(const std::string& robot_name, int index) {
    VCCSim::RGBCameraImageData response = GetRGBIndexedCameraImageData(robot_name, index);
    return std::make_tuple(response.width(), response.height());
}

std::vector<uint8_t> VCCSimClient::GetRGBImageData(const std::string& robot_name, int index, int format) {
    VCCSim::RGBCameraImageData response = GetRGBIndexedCameraImageData(robot_name, index, format);
    return std::vector<uint8_t>(response.data().begin(), response.data().end());
}

bool VCCSimClient::GetAndSaveRGBImage(const std::string& robot_name, int index, 
                                    const std::string& output_path, int format) {
    VCCSim::RGBCameraImageData response = GetRGBIndexedCameraImageData(robot_name, index, format);
    return SaveRGBImage(response, output_path);
}

// LiDAR Service Methods
std::vector<std::tuple<float, float, float>> VCCSimClient::GetLidarData(const std::string& robot_name) {
    VCCSim::LidarData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARData(&context, request, &response);
    
    // Convert response to vector of tuples
    std::vector<std::tuple<float, float, float>> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (const auto& point : response.data()) {
            points.emplace_back(point.x(), point.y(), point.z());
        }
    }
    
    return points;
}

std::tuple<VCCSim::Pose, VCCSim::twist> VCCSimClient::GetLidarOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDAROdom(&context, request, &response);
    
    // Return as tuple
    return std::make_tuple(response.pose(), response.twist());
}

std::tuple<std::vector<std::tuple<float, float, float>>, VCCSim::Odometry> 
VCCSimClient::GetLidarDataAndOdom(const std::string& robot_name) {
    VCCSim::LidarDataAndOdom response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARDataAndOdom(&context, request, &response);
    
    // Convert points to vector of tuples
    std::vector<std::tuple<float, float, float>> points;
    if (status.ok()) {
        points.reserve(response.data().data_size());
        for (const auto& point : response.data().data()) {
            points.emplace_back(point.x(), point.y(), point.z());
        }
    }
    
    return std::make_tuple(points, response.odom());
}

// Depth Camera Service Methods
std::vector<std::tuple<float, float, float>> VCCSimClient::GetDepthCameraPointData(const std::string& robot_name) {
    VCCSim::DepthCameraPointData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraPointData(&context, request, &response);
    
    // Convert response to vector of tuples
    std::vector<std::tuple<float, float, float>> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (const auto& point : response.data()) {
            points.emplace_back(point.x(), point.y(), point.z());
        }
    }
    
    return points;
}

std::vector<float> VCCSimClient::GetDepthCameraImageData(const std::string& robot_name) {
    VCCSim::DepthCameraImageData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraImageData(&context, request, &response);
    
    // Convert response to vector of floats
    std::vector<float> data;
    if (status.ok()) {
        data.reserve(response.data_size());
        for (int i = 0; i < response.data_size(); ++i) {
            data.push_back(response.data(i));
        }
    }
    
    return data;
}

VCCSim::Odometry VCCSimClient::GetDepthCameraOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraOdom(&context, request, &response);
    
    return response;
}

// RGB Camera Service Methods
VCCSim::Odometry VCCSimClient::GetRGBCameraOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBCameraOdom(&context, request, &response);
    
    return response;
}

VCCSim::RGBCameraImageData VCCSimClient::GetRGBIndexedCameraImageData(const std::string& robot_name, int index, 
                                                                    int format) {
    VCCSim::RGBCameraImageData response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::IndexedCamera request;
    auto* robot = request.mutable_robot_name();
    robot->set_name(robot_name);
    request.set_index(index);
    request.set_format(static_cast<VCCSim::Format>(format));  // Cast int to Format enum
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBIndexedCameraImageData(&context, request, &response);
    
    return response;
}

bool VCCSimClient::SaveRGBImage(const VCCSim::RGBCameraImageData& image_data, const std::string& output_path) {
    // Forward to the utility class
    return RGBImageUtils::SaveRGBImage(image_data, output_path);
}

// Drone Service Methods
VCCSim::Pose VCCSimClient::GetDronePose(const std::string& robot_name) {
    VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->GetDronePose(&context, request, &response);
    
    return response;
}

bool VCCSimClient::SendDronePose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::DronePose request;
    request.set_name(name);
    auto* pose = request.mutable_pose();
    pose->set_x(x);
    pose->set_y(y);
    pose->set_z(z);
    pose->set_roll(roll);
    pose->set_pitch(pitch);
    pose->set_yaw(yaw);
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->SendDronePose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendDronePath(const std::string& name, const std::vector<std::tuple<float, float, float, float, float, float>>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::DronePath request;
    request.set_name(name);
    
    for (const auto& pose_tuple : poses) {
        auto* pose = request.add_path();
        pose->set_x(std::get<0>(pose_tuple));
        pose->set_y(std::get<1>(pose_tuple));
        pose->set_z(std::get<2>(pose_tuple));
        pose->set_roll(std::get<3>(pose_tuple));
        pose->set_pitch(std::get<4>(pose_tuple));
        pose->set_yaw(std::get<5>(pose_tuple));
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->SendDronePath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Car Service Methods
VCCSim::Odometry VCCSimClient::GetCarOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->GetCarOdom(&context, request, &response);
    
    return response;
}

bool VCCSimClient::SendCarPose(const std::string& name, float x, float y, float z, float yaw) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::CarPose request;
    request.set_name(name);
    auto* pose = request.mutable_pose();
    pose->set_x(x);
    pose->set_y(y);
    pose->set_z(z);
    pose->set_yaw(yaw);
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->SendCarPose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendCarPath(const std::string& name, const std::vector<std::tuple<float, float, float, float>>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::CarPath request;
    request.set_name(name);
    
    for (const auto& pose_tuple : poses) {
        auto* pose = request.add_path();
        pose->set_x(std::get<0>(pose_tuple));
        pose->set_y(std::get<1>(pose_tuple));
        pose->set_z(std::get<2>(pose_tuple));
        pose->set_yaw(std::get<3>(pose_tuple));
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->SendCarPath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Flash Service Methods
VCCSim::Pose VCCSimClient::GetFlashPose(const std::string& robot_name) {
    VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->GetFlashPose(&context, request, &response);
    
    return response;
}

bool VCCSimClient::SendFlashPose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::FlashPose request;
    request.set_name(name);
    auto* pose = request.mutable_pose();
    pose->set_x(x);
    pose->set_y(y);
    pose->set_z(z);
    pose->set_roll(roll);
    pose->set_pitch(pitch);
    pose->set_yaw(yaw);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->SendFlashPose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendFlashPath(const std::string& name, const std::vector<std::tuple<float, float, float, float, float, float>>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::FlashPath request;
    request.set_name(name);
    
    for (const auto& pose_tuple : poses) {
        auto* pose = request.add_path();
        pose->set_x(std::get<0>(pose_tuple));
        pose->set_y(std::get<1>(pose_tuple));
        pose->set_z(std::get<2>(pose_tuple));
        pose->set_roll(std::get<3>(pose_tuple));
        pose->set_pitch(std::get<4>(pose_tuple));
        pose->set_yaw(std::get<5>(pose_tuple));
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->SendFlashPath(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::CheckFlashReady(const std::string& robot_name) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->CheckFlashReady(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::MoveToNext(const std::string& robot_name) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->MoveToNext(&context, request, &response);
    
    return status.ok() && response.status();
}

// Mesh Service Methods
bool VCCSimClient::SendMesh(const std::string& data, int format, int version, bool simplified,
                           const std::tuple<float, float, float, float, float, float>& transform_pose) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    transform->set_x(std::get<0>(transform_pose));
    transform->set_y(std::get<1>(transform_pose));
    transform->set_z(std::get<2>(transform_pose));
    transform->set_roll(std::get<3>(transform_pose));
    transform->set_pitch(std::get<4>(transform_pose));
    transform->set_yaw(std::get<5>(transform_pose));
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->SendMesh(&context, request, &response);
    
    return status.ok() && response.status();
}

int VCCSimClient::SendGlobalMesh(const std::string& data, int format, int version, bool simplified,
                               const std::tuple<float, float, float, float, float, float>& transform_pose) {
    VCCSim::MeshID response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    transform->set_x(std::get<0>(transform_pose));
    transform->set_y(std::get<1>(transform_pose));
    transform->set_z(std::get<2>(transform_pose));
    transform->set_roll(std::get<3>(transform_pose));
    transform->set_pitch(std::get<4>(transform_pose));
    transform->set_yaw(std::get<5>(transform_pose));
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->SendGlobalMesh(&context, request, &response);
    
    if (status.ok()) {
        return response.id();
    }
    
    return -1; // Error case
}

bool VCCSimClient::RemoveGlobalMesh(int mesh_id) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::MeshID request;
    request.set_id(mesh_id);
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->RemoveGlobalMesh(&context, request, &response);
    
    return status.ok() && response.status();
}

// Point Cloud Service Methods
bool VCCSimClient::SendPointCloudWithColor(const std::vector<std::tuple<float, float, float>>& points,
                                         const std::vector<int>& colors) {
    if (points.size() != colors.size()) {
        // Number of points must match number of colors
        return false;
    }

    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::PointCloudWithColor request;
    
    // Add points and colors
    for (size_t i = 0; i < points.size(); ++i) {
        auto* point_with_color = request.add_data();
        auto* point = point_with_color->mutable_point();
        point->set_x(std::get<0>(points[i]));
        point->set_y(std::get<1>(points[i]));
        point->set_z(std::get<2>(points[i]));
        point_with_color->set_color(colors[i]);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->point_cloud_service_->SendPointCloudWithColor(&context, request, &response);
    
    return status.ok() && response.status();
}

// Forward the helper methods to the implementation
VCCSim::RobotName VCCSimClient::CreateRobotName(const std::string& name) {
    return pImpl->CreateRobotName(name);
}

VCCSim::Pose VCCSimClient::CreatePose(float x, float y, float z, float roll, float pitch, float yaw) {
    return pImpl->CreatePose(x, y, z, roll, pitch, yaw);
}

VCCSim::PoseOnlyYaw VCCSimClient::CreatePoseOnlyYaw(float x, float y, float z, float yaw) {
    return pImpl->CreatePoseOnlyYaw(x, y, z, yaw);
}