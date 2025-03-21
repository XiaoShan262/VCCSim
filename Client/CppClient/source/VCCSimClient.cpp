#include "VCCSimClient.h"

// Include the actual gRPC and protobuf headers only in the implementation file
#include "VCCSim.pb.h"
#include "VCCSim.grpc.pb.h"
#include <grpcpp/grpcpp.h>
#include <fstream>
#include <stdexcept>

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
        lidar_service_ = std::unique_ptr<::VCCSim::LidarService::Stub>(::VCCSim::LidarService::NewStub(channel_).release());
        depth_camera_service_ = std::unique_ptr<::VCCSim::DepthCameraService::Stub>(::VCCSim::DepthCameraService::NewStub(channel_).release());
        rgb_camera_service_ = std::unique_ptr<::VCCSim::RGBCameraService::Stub>(::VCCSim::RGBCameraService::NewStub(channel_).release());
        drone_service_ = std::unique_ptr<::VCCSim::DroneService::Stub>(::VCCSim::DroneService::NewStub(channel_).release());
        car_service_ = std::unique_ptr<::VCCSim::CarService::Stub>(::VCCSim::CarService::NewStub(channel_).release());
        flash_service_ = std::unique_ptr<::VCCSim::FlashService::Stub>(::VCCSim::FlashService::NewStub(channel_).release());
        mesh_service_ = std::unique_ptr<::VCCSim::MeshService::Stub>(::VCCSim::MeshService::NewStub(channel_).release());
        point_cloud_service_ = std::unique_ptr<::VCCSim::PointCloudService::Stub>(::VCCSim::PointCloudService::NewStub(channel_).release());
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
    
    // Conversion functions between our structs and Protocol Buffer classes
    ::VCCSim::RobotName CreateRobotName(const std::string& name) {
        ::VCCSim::RobotName robot_name;
        robot_name.set_name(name);
        return robot_name;
    }
    
    ::VCCSim::Pose ToPbPose(const VCCTypes::Pose& pose) {
        ::VCCSim::Pose pb_pose;
        pb_pose.set_x(pose.x);
        pb_pose.set_y(pose.y);
        pb_pose.set_z(pose.z);
        pb_pose.set_roll(pose.roll);
        pb_pose.set_pitch(pose.pitch);
        pb_pose.set_yaw(pose.yaw);
        return pb_pose;
    }
    
    VCCTypes::Pose FromPbPose(const ::VCCSim::Pose& pb_pose) {
        VCCTypes::Pose pose;
        pose.x = pb_pose.x();
        pose.y = pb_pose.y();
        pose.z = pb_pose.z();
        pose.roll = pb_pose.roll();
        pose.pitch = pb_pose.pitch();
        pose.yaw = pb_pose.yaw();
        return pose;
    }
    
    ::VCCSim::PoseOnlyYaw ToPbPoseYaw(const VCCTypes::PoseYaw& pose) {
        ::VCCSim::PoseOnlyYaw pb_pose;
        pb_pose.set_x(pose.x);
        pb_pose.set_y(pose.y);
        pb_pose.set_z(pose.z);
        pb_pose.set_yaw(pose.yaw);
        return pb_pose;
    }
    
    VCCTypes::PoseYaw FromPbPoseYaw(const ::VCCSim::PoseOnlyYaw& pb_pose) {
        VCCTypes::PoseYaw pose;
        pose.x = pb_pose.x();
        pose.y = pb_pose.y();
        pose.z = pb_pose.z();
        pose.yaw = pb_pose.yaw();
        return pose;
    }
    
    VCCTypes::Twist FromPbTwist(const ::VCCSim::twist& pb_twist) {
        VCCTypes::Twist twist;
        twist.linear_x = pb_twist.linear_x();
        twist.linear_y = pb_twist.linear_y();
        twist.linear_z = pb_twist.linear_z();
        twist.angular_x = pb_twist.angular_x();
        twist.angular_y = pb_twist.angular_y();
        twist.angular_z = pb_twist.angular_z();
        return twist;
    }
    
    VCCTypes::Odometry FromPbOdometry(const ::VCCSim::Odometry& pb_odom) {
        VCCTypes::Odometry odom;
        odom.pose = FromPbPose(pb_odom.pose());
        odom.twist = FromPbTwist(pb_odom.twist());
        return odom;
    }
    
    VCCTypes::RGBImage FromPbRGBImage(const ::VCCSim::RGBCameraImageData& pb_image) {
        VCCTypes::RGBImage image;
        image.width = pb_image.width();
        image.height = pb_image.height();
        image.data = std::vector<uint8_t>(pb_image.data().begin(), pb_image.data().end());
        image.format = static_cast<VCCTypes::Format>(pb_image.format());
        image.timestamp = pb_image.timestamp();
        return image;
    }
    
    VCCTypes::Point FromPbPoint(const ::VCCSim::Point& pb_point) {
        VCCTypes::Point point;
        point.x = pb_point.x();
        point.y = pb_point.y();
        point.z = pb_point.z();
        return point;
    }
    
    ::VCCSim::Point ToPbPoint(const VCCTypes::Point& point) {
        ::VCCSim::Point pb_point;
        pb_point.set_x(point.x);
        pb_point.set_y(point.y);
        pb_point.set_z(point.z);
        return pb_point;
    }
    
    // gRPC channel and stubs
    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<::VCCSim::LidarService::Stub> lidar_service_;
    std::unique_ptr<::VCCSim::DepthCameraService::Stub> depth_camera_service_;
    std::unique_ptr<::VCCSim::RGBCameraService::Stub> rgb_camera_service_;
    std::unique_ptr<::VCCSim::DroneService::Stub> drone_service_;
    std::unique_ptr<::VCCSim::CarService::Stub> car_service_;
    std::unique_ptr<::VCCSim::FlashService::Stub> flash_service_;
    std::unique_ptr<::VCCSim::MeshService::Stub> mesh_service_;
    std::unique_ptr<::VCCSim::PointCloudService::Stub> point_cloud_service_;
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

// RGB Camera Methods

std::vector<uint8_t> VCCSimClient::GetRGBImageData(const std::string& robot_name, int index, VCCTypes::Format format) {
    VCCTypes::RGBImage image = GetRGBIndexedCameraImageData(robot_name, index, format);
    return image.data;
}

bool VCCSimClient::GetAndSaveRGBImage(const std::string& robot_name, int index, 
                                    const std::string& output_path, VCCTypes::Format format) {
    VCCTypes::RGBImage image = GetRGBIndexedCameraImageData(robot_name, index, format);
    return SaveRGBImage(image, output_path);
}

VCCTypes::RGBImage VCCSimClient::GetRGBIndexedCameraImageData(const std::string& robot_name, int index, 
                                                         VCCTypes::Format format) {
    ::VCCSim::RGBCameraImageData response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::IndexedCamera request;
    auto* robot = request.mutable_robot_name();
    robot->set_name(robot_name);
    request.set_index(index);
    request.set_format(static_cast<::VCCSim::Format>(format));
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBIndexedCameraImageData(&context, request, &response);
    
    return pImpl->FromPbRGBImage(response);
}

bool VCCSimClient::SaveRGBImage(const VCCTypes::RGBImage& image_data, const std::string& output_path) {
    try {
        std::ofstream file(output_path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        file.write(reinterpret_cast<const char*>(image_data.data.data()), image_data.data.size());
        file.close();
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

VCCTypes::Odometry VCCSimClient::GetRGBCameraOdom(const std::string& robot_name) {
    ::VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBCameraOdom(&context, request, &response);
    
    return pImpl->FromPbOdometry(response);
}

std::tuple<int, int> VCCSimClient::GetRGBCameraImageSize(const std::string& robot_name, int index) {
    ::VCCSim::ImageSize response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::IndexedCamera request;
    auto* robot = request.mutable_robot_name();
    robot->set_name(robot_name);
    request.set_index(index);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBIndexedCameraImageSize(&context, request, &response);
    
    return std::make_tuple(response.width(), response.height());
}

std::tuple<int, int> VCCSimClient::GetDepthCameraImageSize(const std::string& robot_name) {
    ::VCCSim::ImageSize response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraImageSize(&context, request, &response);
    
    return std::make_tuple(response.width(), response.height());
}

// LiDAR Methods
std::vector<VCCTypes::Point> VCCSimClient::GetLidarData(const std::string& robot_name) {
    ::VCCSim::LidarData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARData(&context, request, &response);
    
    // Convert response to vector of points
    std::vector<VCCTypes::Point> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (const auto& pb_point : response.data()) {
            points.push_back(pImpl->FromPbPoint(pb_point));
        }
    }
    
    return points;
}

VCCTypes::Odometry VCCSimClient::GetLidarOdom(const std::string& robot_name) {
    ::VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDAROdom(&context, request, &response);
    
    return pImpl->FromPbOdometry(response);
}

std::tuple<std::vector<VCCTypes::Point>, VCCTypes::Odometry> 
VCCSimClient::GetLidarDataAndOdom(const std::string& robot_name) {
    ::VCCSim::LidarDataAndOdom response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARDataAndOdom(&context, request, &response);
    
    // Convert points to vector
    std::vector<VCCTypes::Point> points;
    if (status.ok()) {
        points.reserve(response.data().data_size());
        for (const auto& pb_point : response.data().data()) {
            points.push_back(pImpl->FromPbPoint(pb_point));
        }
    }
    
    return std::make_tuple(points, pImpl->FromPbOdometry(response.odom()));
}

// Depth Camera Methods
std::vector<VCCTypes::Point> VCCSimClient::GetDepthCameraPointData(const std::string& robot_name) {
    ::VCCSim::DepthCameraPointData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraPointData(&context, request, &response);
    
    // Convert response to vector of points
    std::vector<VCCTypes::Point> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (const auto& pb_point : response.data()) {
            points.push_back(pImpl->FromPbPoint(pb_point));
        }
    }
    
    return points;
}

std::vector<float> VCCSimClient::GetDepthCameraImageData(const std::string& robot_name) {
    ::VCCSim::DepthCameraImageData response;
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

VCCTypes::Odometry VCCSimClient::GetDepthCameraOdom(const std::string& robot_name) {
    ::VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraOdom(&context, request, &response);
    
    return pImpl->FromPbOdometry(response);
}

// Drone Methods
VCCTypes::Pose VCCSimClient::GetDronePose(const std::string& robot_name) {
    ::VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->GetDronePose(&context, request, &response);
    
    return pImpl->FromPbPose(response);
}

bool VCCSimClient::SendDronePose(const std::string& name, const VCCTypes::Pose& pose) {
    return SendDronePose(name, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

bool VCCSimClient::SendDronePose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::DronePose request;
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

bool VCCSimClient::SendDronePath(const std::string& name, const std::vector<VCCTypes::Pose>& poses) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::DronePath request;
    request.set_name(name);
    
    for (const auto& pose : poses) {
        auto* pb_pose = request.add_path();
        pb_pose->set_x(pose.x);
        pb_pose->set_y(pose.y);
        pb_pose->set_z(pose.z);
        pb_pose->set_roll(pose.roll);
        pb_pose->set_pitch(pose.pitch);
        pb_pose->set_yaw(pose.yaw);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->SendDronePath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Car Methods
VCCTypes::Odometry VCCSimClient::GetCarOdom(const std::string& robot_name) {
    ::VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->GetCarOdom(&context, request, &response);
    
    return pImpl->FromPbOdometry(response);
}

bool VCCSimClient::SendCarPose(const std::string& name, const VCCTypes::PoseYaw& pose) {
    return SendCarPose(name, pose.x, pose.y, pose.z, pose.yaw);
}

bool VCCSimClient::SendCarPose(const std::string& name, float x, float y, float z, float yaw) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::CarPose request;
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

bool VCCSimClient::SendCarPath(const std::string& name, const std::vector<VCCTypes::PoseYaw>& poses) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::CarPath request;
    request.set_name(name);
    
    for (const auto& pose : poses) {
        auto* pb_pose = request.add_path();
        pb_pose->set_x(pose.x);
        pb_pose->set_y(pose.y);
        pb_pose->set_z(pose.z);
        pb_pose->set_yaw(pose.yaw);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->SendCarPath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Flash Methods
VCCTypes::Pose VCCSimClient::GetFlashPose(const std::string& robot_name) {
    ::VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->GetFlashPose(&context, request, &response);
    
    return pImpl->FromPbPose(response);
}

bool VCCSimClient::SendFlashPose(const std::string& name, const VCCTypes::Pose& pose) {
    return SendFlashPose(name, pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
}

bool VCCSimClient::SendFlashPose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::FlashPose request;
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

bool VCCSimClient::SendFlashPath(const std::string& name, const std::vector<VCCTypes::Pose>& poses) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::FlashPath request;
    request.set_name(name);
    
    for (const auto& pose : poses) {
        auto* pb_pose = request.add_path();
        pb_pose->set_x(pose.x);
        pb_pose->set_y(pose.y);
        pb_pose->set_z(pose.z);
        pb_pose->set_roll(pose.roll);
        pb_pose->set_pitch(pose.pitch);
        pb_pose->set_yaw(pose.yaw);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->SendFlashPath(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::CheckFlashReady(const std::string& robot_name) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->CheckFlashReady(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::MoveToNext(const std::string& robot_name) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->MoveToNext(&context, request, &response);
    
    return status.ok() && response.status();
}

// Mesh Methods
bool VCCSimClient::SendMesh(const std::string& data, int format, int version, bool simplified,
                          const VCCTypes::Pose& transform_pose) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    transform->set_x(transform_pose.x);
    transform->set_y(transform_pose.y);
    transform->set_z(transform_pose.z);
    transform->set_roll(transform_pose.roll);
    transform->set_pitch(transform_pose.pitch);
    transform->set_yaw(transform_pose.yaw);
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->SendMesh(&context, request, &response);
    
    return status.ok() && response.status();
}

int VCCSimClient::SendGlobalMesh(const std::string& data, int format, int version, bool simplified,
                               const VCCTypes::Pose& transform_pose) {
    ::VCCSim::MeshID response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    transform->set_x(transform_pose.x);
    transform->set_y(transform_pose.y);
    transform->set_z(transform_pose.z);
    transform->set_roll(transform_pose.roll);
    transform->set_pitch(transform_pose.pitch);
    transform->set_yaw(transform_pose.yaw);
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->SendGlobalMesh(&context, request, &response);
    
    if (status.ok()) {
        return response.id();
    }
    
    return -1; // Error case
}

bool VCCSimClient::RemoveGlobalMesh(int mesh_id) {
    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::MeshID request;
    request.set_id(mesh_id);
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->RemoveGlobalMesh(&context, request, &response);
    
    return status.ok() && response.status();
}

// Point Cloud Methods
bool VCCSimClient::SendPointCloudWithColor(const std::vector<VCCTypes::Point>& points,
                                         const std::vector<int>& colors) {
    if (points.size() != colors.size()) {
        // Number of points must match number of colors
        return false;
    }

    ::VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    ::VCCSim::PointCloudWithColor request;
    
    // Add points and colors
    for (size_t i = 0; i < points.size(); ++i) {
        auto* point_with_color = request.add_data();
        auto* pb_point = point_with_color->mutable_point();
        pb_point->set_x(points[i].x);
        pb_point->set_y(points[i].y);
        pb_point->set_z(points[i].z);
        point_with_color->set_color(colors[i]);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->point_cloud_service_->SendPointCloudWithColor(&context, request, &response);
    
    return status.ok() && response.status();
}