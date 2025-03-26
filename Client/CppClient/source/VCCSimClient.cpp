#include "VCCSimClient.h"

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
    
    // Helper function to create a RobotName message
    ::VCCSim::RobotName CreateRobotName(const std::string& name) {
        ::VCCSim::RobotName robot_name;
        robot_name.set_name(name);
        return robot_name;
    }

    // Transform point from UE (left-handed, cm) to right-handed (m)
    VCCSim::Point TransformPointFromUE(const VCCSim::Point& ue_point) {
        VCCSim::Point transformed;
        // Convert cm to m and flip Y-axis for right-handed system
        transformed.set_x(ue_point.x() * 0.01f);
        transformed.set_y(-ue_point.y() * 0.01f);
        transformed.set_z(ue_point.z() * 0.01f);
        return transformed;
    }

    // Transform point to UE (left-handed, cm) from right-handed (m)
    VCCSim::Point TransformPointToUE(const VCCSim::Point& point) {
        VCCSim::Point transformed;
        // Convert m to cm and flip Y-axis for left-handed system
        transformed.set_x(point.x() * 100.0f);
        transformed.set_y(-point.y() * 100.0f);
        transformed.set_z(point.z() * 100.0f);
        return transformed;
    }

    // Transform pose from UE (left-handed, cm) to right-handed (m)
    VCCSim::Pose TransformPoseFromUE(const VCCSim::Pose& ue_pose) {
        VCCSim::Pose transformed;
        // Convert cm to m and flip Y-axis for right-handed system
        transformed.set_x(ue_pose.x() * 0.01f);
        transformed.set_y(-ue_pose.y() * 0.01f);
        transformed.set_z(ue_pose.z() * 0.01f);
        
        // For rotations, we need to adjust roll, pitch, yaw for coordinate system change
        // In a Y-axis flip, roll and yaw change sign, pitch remains the same
        transformed.set_roll(-ue_pose.roll());
        transformed.set_pitch(ue_pose.pitch());
        transformed.set_yaw(-ue_pose.yaw());
        
        return transformed;
    }

    // Transform pose to UE (left-handed, cm) from right-handed (m)
    VCCSim::Pose TransformPoseToUE(const VCCSim::Pose& pose) {
        VCCSim::Pose transformed;
        // Convert m to cm and flip Y-axis for left-handed system
        transformed.set_x(pose.x() * 100.0f);
        transformed.set_y(-pose.y() * 100.0f);
        transformed.set_z(pose.z() * 100.0f);
        
        // For rotations, invert the same angles as in TransformPoseFromUE
        transformed.set_roll(-pose.roll());
        transformed.set_pitch(pose.pitch());
        transformed.set_yaw(-pose.yaw());
        
        return transformed;
    }

    // Transform pose with only yaw from UE (left-handed, cm) to right-handed (m)
    VCCSim::PoseOnlyYaw TransformPoseOnlyYawFromUE(const VCCSim::PoseOnlyYaw& ue_pose) {
        VCCSim::PoseOnlyYaw transformed;
        // Convert cm to m and flip Y-axis for right-handed system
        transformed.set_x(ue_pose.x() * 0.01f);
        transformed.set_y(-ue_pose.y() * 0.01f);
        transformed.set_z(ue_pose.z() * 0.01f);
        
        // Flip yaw for coordinate system change
        transformed.set_yaw(-ue_pose.yaw());
        
        return transformed;
    }

    // Transform pose with only yaw to UE (left-handed, cm) from right-handed (m)
    VCCSim::PoseOnlyYaw TransformPoseOnlyYawToUE(const VCCSim::PoseOnlyYaw& pose) {
        VCCSim::PoseOnlyYaw transformed;
        // Convert m to cm and flip Y-axis for left-handed system
        transformed.set_x(pose.x() * 100.0f);
        transformed.set_y(-pose.y() * 100.0f);
        transformed.set_z(pose.z() * 100.0f);
        
        // Flip yaw for coordinate system change
        transformed.set_yaw(-pose.yaw());
        
        return transformed;
    }

    // Transform odometry from UE (left-handed, cm) to right-handed (m)
    VCCSim::Odometry TransformOdometryFromUE(const VCCSim::Odometry& ue_odom) {
        VCCSim::Odometry transformed;
        
        // Transform pose
        auto* pose = transformed.mutable_pose();
        pose->set_x(ue_odom.pose().x() * 0.01f);
        pose->set_y(-ue_odom.pose().y() * 0.01f);
        pose->set_z(ue_odom.pose().z() * 0.01f);
        pose->set_roll(-ue_odom.pose().roll());
        pose->set_pitch(ue_odom.pose().pitch());
        pose->set_yaw(-ue_odom.pose().yaw());
        
        // Transform twist (linear and angular velocities)
        auto* twist = transformed.mutable_twist();
        
        // Linear velocity (cm/s to m/s, flip Y)
        twist->set_linear_x(ue_odom.twist().linear_x() * 0.01f);
        twist->set_linear_y(-ue_odom.twist().linear_y() * 0.01f);
        twist->set_linear_z(ue_odom.twist().linear_z() * 0.01f);
        
        // Angular velocity (flip for roll and yaw due to coordinate system change)
        twist->set_angular_x(-ue_odom.twist().angular_x());
        twist->set_angular_y(ue_odom.twist().angular_y());
        twist->set_angular_z(-ue_odom.twist().angular_z());
        
        return transformed;
    }

    // Transform odometry to UE (left-handed, cm) from right-handed (m)
    VCCSim::Odometry TransformOdometryToUE(const VCCSim::Odometry& odom) {
        VCCSim::Odometry transformed;
        
        // Transform pose
        auto* pose = transformed.mutable_pose();
        pose->set_x(odom.pose().x() * 100.0f);
        pose->set_y(-odom.pose().y() * 100.0f);
        pose->set_z(odom.pose().z() * 100.0f);
        pose->set_roll(-odom.pose().roll());
        pose->set_pitch(odom.pose().pitch());
        pose->set_yaw(-odom.pose().yaw());
        
        // Transform twist (linear and angular velocities)
        auto* twist = transformed.mutable_twist();

        // Linear velocity (m/s to cm/s, flip Y)
        twist->set_linear_x(odom.twist().linear_x() * 100.0f);
        twist->set_linear_y(-odom.twist().linear_y() * 100.0f);
        twist->set_linear_z(odom.twist().linear_z() * 100.0f);

        // Angular velocity (flip for roll and yaw due to coordinate system change)
        twist->set_angular_x(-odom.twist().angular_x());
        twist->set_angular_y(odom.twist().angular_y());
        twist->set_angular_z(-odom.twist().angular_z());
        
        return transformed;
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

std::vector<uint8_t> VCCSimClient::GetRGBImageData(const std::string& robot_name, int index, VCCSim::Format format) {
    VCCSim::RGBCameraImageData image = GetRGBIndexedCameraImageData(robot_name, index, format);
    return std::vector<uint8_t>(image.data().begin(), image.data().end());
}

bool VCCSimClient::GetAndSaveRGBImage(const std::string& robot_name, int index, 
                                    const std::string& output_path, VCCSim::Format format) {
    VCCSim::RGBCameraImageData image = GetRGBIndexedCameraImageData(robot_name, index, format);
    return SaveRGBImage(image, output_path);
}

VCCSim::RGBCameraImageData VCCSimClient::GetRGBIndexedCameraImageData(const std::string& robot_name, int index, 
                                                         VCCSim::Format format) {
    VCCSim::RGBCameraImageData response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::IndexedCamera request;
    auto* robot = request.mutable_robot_name();
    robot->set_name(robot_name);
    request.set_index(index);
    request.set_format(format);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBIndexedCameraImageData(&context, request, &response);
    
    return response;
}

bool VCCSimClient::SaveRGBImage(const VCCSim::RGBCameraImageData& image_data, const std::string& output_path) {
    try {
        std::ofstream file(output_path, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        file.write(image_data.data().data(), image_data.data().size());
        file.close();
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

VCCSim::Odometry VCCSimClient::GetRGBCameraOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBCameraOdom(&context, request, &response);
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformOdometryFromUE(response);
    }
    
    return response;
}

std::tuple<int, int> VCCSimClient::GetRGBCameraImageSize(const std::string& robot_name, int index) {
    VCCSim::ImageSize response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::IndexedCamera request;
    auto* robot = request.mutable_robot_name();
    robot->set_name(robot_name);
    request.set_index(index);
    
    // Call gRPC method
    grpc::Status status = pImpl->rgb_camera_service_->GetRGBIndexedCameraImageSize(&context, request, &response);
    
    return std::make_tuple(response.width(), response.height());
}

std::tuple<int, int> VCCSimClient::GetDepthCameraImageSize(const std::string& robot_name) {
    VCCSim::ImageSize response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraImageSize(&context, request, &response);
    
    return std::make_tuple(response.width(), response.height());
}

// LiDAR Methods
std::vector<VCCSim::Point> VCCSimClient::GetLidarData(const std::string& robot_name) {
    VCCSim::LidarData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARData(&context, request, &response);
    
    // Convert response to vector of points with coordinate transformation
    std::vector<VCCSim::Point> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (int i = 0; i < response.data_size(); ++i) {
            points.push_back(pImpl->TransformPointFromUE(response.data(i)));
        }
    }
    
    return points;
}

VCCSim::Odometry VCCSimClient::GetLidarOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDAROdom(&context, request, &response);
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformOdometryFromUE(response);
    }
    
    return response;
}

std::tuple<std::vector<VCCSim::Point>, VCCSim::Odometry> 
VCCSimClient::GetLidarDataAndOdom(const std::string& robot_name) {
    VCCSim::LidarDataAndOdom response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->lidar_service_->GetLiDARDataAndOdom(&context, request, &response);
    
    // Convert points to vector with coordinate transformation
    std::vector<VCCSim::Point> points;
    VCCSim::Odometry transformed_odom;
    
    if (status.ok()) {
        // Transform points
        points.reserve(response.data().data_size());
        for (int i = 0; i < response.data().data_size(); ++i) {
            points.push_back(pImpl->TransformPointFromUE(response.data().data(i)));
        }
        
        // Transform odometry
        transformed_odom = pImpl->TransformOdometryFromUE(response.odom());
    }
    
    return std::make_tuple(points, transformed_odom);
}

// Depth Camera Methods
std::vector<VCCSim::Point> VCCSimClient::GetDepthCameraPointData(const std::string& robot_name) {
    VCCSim::DepthCameraPointData response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->depth_camera_service_->GetDepthCameraPointData(&context, request, &response);
    
    // Convert response to vector of points with coordinate transformation
    std::vector<VCCSim::Point> points;
    if (status.ok()) {
        points.reserve(response.data_size());
        for (int i = 0; i < response.data_size(); ++i) {
            points.push_back(pImpl->TransformPointFromUE(response.data(i)));
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
    
    // Convert response to vector of floats, transforming depth values from cm to m
    std::vector<float> data;
    if (status.ok()) {
        data.reserve(response.data_size());
        for (int i = 0; i < response.data_size(); ++i) {
            // Convert from cm to m
            data.push_back(response.data(i) * 0.01f);
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
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformOdometryFromUE(response);
    }
    
    return response;
}

// Drone Methods
VCCSim::Pose VCCSimClient::GetDronePose(const std::string& robot_name) {
    VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->GetDronePose(&context, request, &response);
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformPoseFromUE(response);
    }
    
    return response;
}

bool VCCSimClient::SendDronePose(const std::string& name, const VCCSim::Pose& pose) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Transform to UE coordinates
    VCCSim::Pose ue_pose = pImpl->TransformPoseToUE(pose);
    
    // Create request
    VCCSim::DronePose request;
    request.set_name(name);
    auto* pose_ptr = request.mutable_pose();
    *pose_ptr = ue_pose;
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->SendDronePose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendDronePose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    // Create a pose and use the existing method
    VCCSim::Pose pose;
    pose.set_x(x);
    pose.set_y(y);
    pose.set_z(z);
    pose.set_roll(roll);
    pose.set_pitch(pitch);
    pose.set_yaw(yaw);
    
    return SendDronePose(name, pose);
}

bool VCCSimClient::SendDronePath(const std::string& name, const std::vector<VCCSim::Pose>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::DronePath request;
    request.set_name(name);
    
    // Transform each pose to UE coordinates
    for (const auto& pose : poses) {
        auto ue_pose = pImpl->TransformPoseToUE(pose);
        auto* pb_pose = request.add_path();
        *pb_pose = ue_pose;
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->drone_service_->SendDronePath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Car Methods
VCCSim::Odometry VCCSimClient::GetCarOdom(const std::string& robot_name) {
    VCCSim::Odometry response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->GetCarOdom(&context, request, &response);
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformOdometryFromUE(response);
    }
    
    return response;
}

bool VCCSimClient::SendCarPose(const std::string& name, const VCCSim::PoseOnlyYaw& pose) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Transform to UE coordinates
    VCCSim::PoseOnlyYaw ue_pose = pImpl->TransformPoseOnlyYawToUE(pose);
    
    // Create request
    VCCSim::CarPose request;
    request.set_name(name);
    auto* pose_ptr = request.mutable_pose();
    *pose_ptr = ue_pose;
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->SendCarPose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendCarPose(const std::string& name, float x, float y, float z, float yaw) {
    // Create a pose and use the existing method
    VCCSim::PoseOnlyYaw pose;
    pose.set_x(x);
    pose.set_y(y);
    pose.set_z(z);
    pose.set_yaw(yaw);
    
    return SendCarPose(name, pose);
}

bool VCCSimClient::SendCarPath(const std::string& name, const std::vector<VCCSim::PoseOnlyYaw>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::CarPath request;
    request.set_name(name);
    
    // Transform each pose to UE coordinates
    for (const auto& pose : poses) {
        auto ue_pose = pImpl->TransformPoseOnlyYawToUE(pose);
        auto* pb_pose = request.add_path();
        *pb_pose = ue_pose;
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->car_service_->SendCarPath(&context, request, &response);
    
    return status.ok() && response.status();
}

// Flash Methods
VCCSim::Pose VCCSimClient::GetFlashPose(const std::string& robot_name) {
    VCCSim::Pose response;
    grpc::ClientContext context;
    
    // Create request
    auto request = pImpl->CreateRobotName(robot_name);
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->GetFlashPose(&context, request, &response);
    
    // Transform from UE coordinates
    if (status.ok()) {
        return pImpl->TransformPoseFromUE(response);
    }
    
    return response;
}

bool VCCSimClient::SendFlashPose(const std::string& name, const VCCSim::Pose& pose) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Transform to UE coordinates
    VCCSim::Pose ue_pose = pImpl->TransformPoseToUE(pose);
    
    // Create request
    VCCSim::FlashPose request;
    request.set_name(name);
    auto* pose_ptr = request.mutable_pose();
    *pose_ptr = ue_pose;
    
    // Call gRPC method
    grpc::Status status = pImpl->flash_service_->SendFlashPose(&context, request, &response);
    
    return status.ok() && response.status();
}

bool VCCSimClient::SendFlashPose(const std::string& name, float x, float y, float z, float roll, float pitch, float yaw) {
    // Create a pose and use the existing method
    VCCSim::Pose pose;
    pose.set_x(x);
    pose.set_y(y);
    pose.set_z(z);
    pose.set_roll(roll);
    pose.set_pitch(pitch);
    pose.set_yaw(yaw);
    
    return SendFlashPose(name, pose);
}

bool VCCSimClient::SendFlashPath(const std::string& name, const std::vector<VCCSim::Pose>& poses) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::FlashPath request;
    request.set_name(name);
    
    // Transform each pose to UE coordinates
    for (const auto& pose : poses) {
        auto ue_pose = pImpl->TransformPoseToUE(pose);
        auto* pb_pose = request.add_path();
        *pb_pose = ue_pose;
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

// Mesh Methods
bool VCCSimClient::SendMesh(const std::string& data, int format, int version, bool simplified,
                          const VCCSim::Pose& transform_pose) {
    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Transform pose to UE coordinates
    VCCSim::Pose ue_transform_pose = pImpl->TransformPoseToUE(transform_pose);
    
    // Create request
    VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    *transform = ue_transform_pose;
    
    // Call gRPC method
    grpc::Status status = pImpl->mesh_service_->SendMesh(&context, request, &response);
    
    return status.ok() && response.status();
}

int VCCSimClient::SendGlobalMesh(const std::string& data, int format, int version, bool simplified,
                               const VCCSim::Pose& transform_pose) {
    VCCSim::MeshID response;
    grpc::ClientContext context;
    
    // Transform pose to UE coordinates
    VCCSim::Pose ue_transform_pose = pImpl->TransformPoseToUE(transform_pose);
    
    // Create request
    VCCSim::MeshData request;
    request.set_data(data);
    request.set_format(format);
    request.set_version(version);
    request.set_simplified(simplified);
    
    // Set transform pose
    auto* transform = request.mutable_transform();
    *transform = ue_transform_pose;
    
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

// Point Cloud Methods
bool VCCSimClient::SendPointCloudWithColor(const std::vector<VCCSim::Point>& points,
                                         const std::vector<int>& colors) {
    if (points.size() != colors.size()) {
        // Number of points must match number of colors
        return false;
    }

    VCCSim::Status response;
    grpc::ClientContext context;
    
    // Create request
    VCCSim::PointCloudWithColor request;
    
    // Add points and colors, transforming each point to UE coordinates
    for (size_t i = 0; i < points.size(); ++i) {
        auto* point_with_color = request.add_data();
        auto* pb_point = point_with_color->mutable_point();
        
        // Transform point to UE coordinates
        VCCSim::Point ue_point = pImpl->TransformPointToUE(points[i]);
        *pb_point = ue_point;
        
        point_with_color->set_color(colors[i]);
    }
    
    // Call gRPC method
    grpc::Status status = pImpl->point_cloud_service_->SendPointCloudWithColor(&context, request, &response);
    
    return status.ok() && response.status();
}