#include "API/GRPCCall.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/CameraSensor.h"
#include "Utils/MeshHandlerComponent.h"
#include "Utils/InsMeshHolder.h"
#include "Pawns/DronePawn.h"

LidarGetDataCall::LidarGetDataCall(VCCSim::LidarService::AsyncService* service,
    grpc::ServerCompletionQueue* cq, ULidarComponent* lidar_component)
    : AsyncCallTemplate(service, cq, lidar_component) {
    Proceed(true);
}

void LidarGetDataCall::PrepareNextCall() {
    new LidarGetDataCall(service_, cq_, component_);
}

void LidarGetDataCall::InitializeRequest() {
    service_->RequestGetLiDARData(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void LidarGetDataCall::ProcessRequest() {
    if (component_) {
        TArray<FLidarPoint> Data = component_->GetPointCloudData();
        for (const auto& Point : Data) {
            VCCSim::LidarPoint* LidarPoint = response_.add_data();
            LidarPoint->set_x(Point.Location.X);
            LidarPoint->set_y(Point.Location.Y);
            LidarPoint->set_z(Point.Location.Z);
            LidarPoint->set_hit(Point.bHit);
        }
    }
    else {
        UE_LOG(LogTemp, Error, TEXT("Lidar component not found!"));
    }
}

LidarGetOdomCall::LidarGetOdomCall(VCCSim::LidarService::AsyncService* service,
    grpc::ServerCompletionQueue* cq, ULidarComponent* lidar_component)
    : AsyncCallTemplate(service, cq, lidar_component) {
    Proceed(true);
}

void LidarGetOdomCall::PrepareNextCall() {
    new LidarGetOdomCall(service_, cq_, component_);
}

void LidarGetOdomCall::InitializeRequest() {
    service_->RequestGetLiDAROdom(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void LidarGetOdomCall::ProcessRequest() {
    if (component_) {
        const FVector Pose = component_->GetComponentLocation();
        const FRotator Rot = component_->GetComponentRotation();
        const FVector LinearVelocity = component_->GetPhysicsLinearVelocity();
        const FVector AngularVelocity = component_->GetPhysicsAngularVelocityInDegrees();
        
        VCCSim::Pose* PoseData = response_.mutable_pose();
        PoseData->set_x(Pose.X);
        PoseData->set_y(Pose.Y);
        PoseData->set_z(Pose.Z);
        PoseData->set_roll(Rot.Roll);
        PoseData->set_pitch(Rot.Pitch);
        PoseData->set_yaw(Rot.Yaw);
        
        VCCSim::twist* TwistData = response_.mutable_twist();
        TwistData->set_linear_x(LinearVelocity.X);
        TwistData->set_linear_y(LinearVelocity.Y);
        TwistData->set_linear_z(LinearVelocity.Z);
        TwistData->set_angular_x(AngularVelocity.X);
        TwistData->set_angular_y(AngularVelocity.Y);
        TwistData->set_angular_z(AngularVelocity.Z);
    }
    else {
        UE_LOG(LogTemp, Error, TEXT("Lidar component not found!"));
    }
}

LidarGetDataAndOdomCall::LidarGetDataAndOdomCall(
    VCCSim::LidarService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, ULidarComponent*> rcmap)
    : AsyncCallTemplateM(service, cq, rcmap)
{
    Proceed(true);
}

void LidarGetDataAndOdomCall::PrepareNextCall()
{
    new LidarGetDataAndOdomCall(service_, cq_, RCMap_);
}

void LidarGetDataAndOdomCall::InitializeRequest()
{
    service_->RequestGetLiDARDataAndOdom(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void LidarGetDataAndOdomCall::ProcessRequest()
{
    if (RCMap_.contains(request_.name()))
    {
        auto DataAndOdom =
            RCMap_[request_.name()]->GetPointCloudDataAndOdom();
        const auto& Odom = DataAndOdom.Value;
        
        for (const auto& Point : DataAndOdom.Key) {
            VCCSim::LidarPoint* LidarPoint = response_.mutable_data()->add_data();
            LidarPoint->set_x(Point.Location.X);
            LidarPoint->set_y(Point.Location.Y);
            LidarPoint->set_z(Point.Location.Z);
            LidarPoint->set_hit(Point.bHit);
        }
        
        VCCSim::Pose* PoseData = response_.mutable_odom()->mutable_pose();
        PoseData->set_x(Odom.Location.X);
        PoseData->set_y(Odom.Location.Y);
        PoseData->set_z(Odom.Location.Z);
        PoseData->set_roll(Odom.Rotation.Roll);
        PoseData->set_pitch(Odom.Rotation.Pitch);
        PoseData->set_yaw(Odom.Rotation.Yaw);
        
        VCCSim::twist* TwistData = response_.mutable_odom()->mutable_twist();
        TwistData->set_linear_x(Odom.LinearVelocity.X);
        TwistData->set_linear_y(Odom.LinearVelocity.Y);
        TwistData->set_linear_z(Odom.LinearVelocity.Z);
        TwistData->set_angular_x(Odom.AngularVelocity.X);
        TwistData->set_angular_y(Odom.AngularVelocity.Y);
        TwistData->set_angular_z(Odom.AngularVelocity.Z);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("LidarGetDataAndOdomCall: Lidar component not found!"));
    }
}

DepthCameraGetPointDataCall::DepthCameraGetPointDataCall(
    VCCSim::DepthCameraService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, UDepthCameraComponent*> rdcmap)
        : AsyncCallTemplateM(service, cq, rdcmap)
{
    Proceed(true);
}

void DepthCameraGetPointDataCall::PrepareNextCall()
{
    new DepthCameraGetPointDataCall(service_, cq_, RCMap_);
}

void DepthCameraGetPointDataCall::InitializeRequest()
{
    service_->RequestGetDepthCameraPointData(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void DepthCameraGetPointDataCall::ProcessRequest()
{
    Promise = MakeShared<TPromise<void>>();
    auto Future = Promise->GetFuture();

    if (!RCMap_.contains(request_.name()))
    {
        UE_LOG(LogTemp, Warning, TEXT("DepthCameraGetPointDataCall: "
                                      "DepthCamera component not found!"));
        Promise->SetValue();  // Complete the promise even in error case
        return;
    }

    auto* DepthCamera = RCMap_[request_.name()];
    if (!DepthCamera)
    {
        UE_LOG(LogTemp, Warning, TEXT("DepthCameraGetPointDataCall: "
                                      "Invalid DepthCamera reference!"));
        Promise->SetValue();
        return;
    }

    // Capture WeakPtr to Promise to avoid lifetime issues
    TWeakPtr<TPromise<void>> WeakPromise = Promise;
    
    DepthCamera->AsyncGetPointCloudData(
        [this, WeakPromise](TArray<FDCPoint> Points) {
        auto StrongPromise = WeakPromise.Pin();
        if (!StrongPromise)
        {
            UE_LOG(LogTemp, Error,
                TEXT("Promise was destroyed before completion"));
            return;
        }

        // Convert points to protobuf format
        for (const auto& Point : Points)
        {
            VCCSim::Point* DepthPoint = response_.add_data();
            DepthPoint->set_x(Point.Location.X);
            DepthPoint->set_y(Point.Location.Y);
            DepthPoint->set_z(Point.Location.Z);
        }
        
        StrongPromise->SetValue();
    });

    // Wait with timeout
    const float TimeoutSeconds = 5.0f;
    if (!Future.WaitFor(FTimespan::FromSeconds(TimeoutSeconds)))
    {
        UE_LOG(LogTemp, Error, TEXT("GetPointCloudData timed out after "
                                    "%f seconds"), TimeoutSeconds);
        if (Promise.IsValid())
        {
            Promise->SetValue();  // Ensure promise is completed
        }
    }
}

DepthCameraGetImageDataCall::DepthCameraGetImageDataCall(
    VCCSim::DepthCameraService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, UDepthCameraComponent*> rdcmap)
        : AsyncCallTemplateM(service, cq, rdcmap)
{
    Proceed(true);
}

void DepthCameraGetImageDataCall::PrepareNextCall()
{
    new DepthCameraGetImageDataCall(service_, cq_, RCMap_);
}

void DepthCameraGetImageDataCall::InitializeRequest()
{
    service_->RequestGetDepthCameraImageData(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void DepthCameraGetImageDataCall::ProcessRequest()
{
    Promise = MakeShared<TPromise<void>>();
    auto Future = Promise->GetFuture();

    if (!RCMap_.contains(request_.name()))
    {
        UE_LOG(LogTemp, Warning, TEXT("DepthCameraGetImageDataCall: "
                                      "DepthCamera component not found!"));
        Promise->SetValue();
        return;
    }

    auto* DepthCamera = RCMap_[request_.name()];
    if (!DepthCamera)
    {
        UE_LOG(LogTemp, Warning, TEXT("DepthCameraGetImageDataCall: "
                                      "Invalid DepthCamera reference!"));
        Promise->SetValue();
        return;
    }

    // Capture WeakPtr to Promise to avoid lifetime issues
    TWeakPtr<TPromise<void>> WeakPromise = Promise;

    DepthCamera->AsyncGetDepthImageData(
        [this, WeakPromise](TArray<float> DepthImage) {
        auto StrongPromise = WeakPromise.Pin();
        if (!StrongPromise)
        {
            UE_LOG(LogTemp, Error,
                TEXT("Promise was destroyed before completion"));
            return;
        }

        // Convert depth image to protobuf format
        for (const float& Depth : DepthImage)
        {
            response_.add_data(Depth);
        }
        
        StrongPromise->SetValue();
    });

    // Wait with timeout
    const float TimeoutSeconds = 5.0f;
    if (!Future.WaitFor(FTimespan::FromSeconds(TimeoutSeconds)))
    {
        UE_LOG(LogTemp, Error, TEXT("GetDepthImageData timed out after "
                                    "%f seconds"), TimeoutSeconds);
        if (Promise.IsValid())
        {
            Promise->SetValue();  // Ensure promise is completed
        }
    }
}

DepthCameraGetOdomCall::DepthCameraGetOdomCall(
    VCCSim::DepthCameraService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, UDepthCameraComponent*> rdcmap)
        : AsyncCallTemplateM(service, cq, rdcmap)
{
    Proceed(true);
}

void DepthCameraGetOdomCall::PrepareNextCall()
{
    new DepthCameraGetOdomCall(service_, cq_, RCMap_);
}

void DepthCameraGetOdomCall::InitializeRequest()
{
    service_->RequestGetDepthCameraOdom(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void DepthCameraGetOdomCall::ProcessRequest()
{
    if (RCMap_.contains(request_.name()))
    {
        const FVector Pose = RCMap_[request_.name()]->GetComponentLocation();
        const FRotator Rot = RCMap_[request_.name()]->GetComponentRotation();
        const FVector LinearVelocity = RCMap_[request_.name()]->GetPhysicsLinearVelocity();
        const FVector AngularVelocity = RCMap_[request_.name()]->GetPhysicsAngularVelocityInDegrees();

        VCCSim::Pose* PoseData = response_.mutable_pose();
        PoseData->set_x(Pose.X);
        PoseData->set_y(Pose.Y);
        PoseData->set_z(Pose.Z);
        PoseData->set_roll(Rot.Roll);
        PoseData->set_pitch(Rot.Pitch);
        PoseData->set_yaw(Rot.Yaw);

        VCCSim::twist* TwistData = response_.mutable_twist();
        TwistData->set_linear_x(LinearVelocity.X);
        TwistData->set_linear_y(LinearVelocity.Y);
        TwistData->set_linear_z(LinearVelocity.Z);
        TwistData->set_angular_x(AngularVelocity.X);
        TwistData->set_angular_y(AngularVelocity.Y);
        TwistData->set_angular_z(AngularVelocity.Z);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("DepthCameraGetOdomCall: "
                                      "DepthCamera component not found!"));
    }
}

RGBCameraGetImageDataCall::RGBCameraGetImageDataCall(
    VCCSim::RGBCameraService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, URGBCameraComponent*> rrgbcmap)
        : AsyncCallTemplateM(service, cq, rrgbcmap)
{
    Proceed(true);
}

void RGBCameraGetImageDataCall::PrepareNextCall()
{
    new RGBCameraGetImageDataCall(service_, cq_, RCMap_);
}

void RGBCameraGetImageDataCall::InitializeRequest()
{
    service_->RequestGetRGBCameraImageData(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void RGBCameraGetImageDataCall::ProcessRequest()
{
    Promise = MakeShared<TPromise<void>>();
    auto Future = Promise->GetFuture();

    if (!RCMap_.contains(request_.name()))
    {
        UE_LOG(LogTemp, Warning, TEXT("RGBCameraGetImageDataCall: "
                                    "RGB Camera component not found!"));
        Promise->SetValue();
        return;
    }

    auto* RGBCamera = RCMap_[request_.name()];
    if (!RGBCamera)
    {
        UE_LOG(LogTemp, Warning, TEXT("RGBCameraGetImageDataCall: "
                                    "Invalid RGB Camera reference!"));
        Promise->SetValue();
        return;
    }

    // Capture both WeakPtr to Promise and RGBCamera
    TWeakPtr<TPromise<void>> WeakPromise = Promise;
    TWeakObjectPtr<URGBCameraComponent> WeakRGBCamera(RGBCamera);

    RGBCamera->AsyncGetRGBImageData([this, WeakPromise, WeakRGBCamera](TArray<FRGBPixel> RGBImage) {
        auto StrongPromise = WeakPromise.Pin();
        auto StrongRGBCamera = WeakRGBCamera.Get();
        
        if (!StrongPromise)
        {
            UE_LOG(LogTemp, Error, TEXT("Promise was destroyed before completion"));
            return;
        }

        if (!StrongRGBCamera)
        {
            UE_LOG(LogTemp, Error, TEXT("RGB Camera was destroyed before completion"));
            StrongPromise->SetValue();
            return;
        }

        // Pack RGB data into bytes
        const int32 NumPixels = RGBImage.Num();
        const int32 BytesPerPixel = 3; // RGB
        TArray<uint8> PackedData;
        PackedData.SetNum(NumPixels * BytesPerPixel);

        for (int32 i = 0; i < NumPixels; ++i)
        {
            const FRGBPixel& Pixel = RGBImage[i];
            const int32 BaseIndex = i * BytesPerPixel;
            PackedData[BaseIndex] = Pixel.R;
            PackedData[BaseIndex + 1] = Pixel.G;
            PackedData[BaseIndex + 2] = Pixel.B;
        }

        response_.set_width(StrongRGBCamera->Width);
        response_.set_height(StrongRGBCamera->Height);
        response_.set_format(VCCSim::RGBCameraImageData::RGB);
        response_.set_data(PackedData.GetData(), PackedData.Num());
        
        StrongPromise->SetValue();
    });

    // Wait with timeout
    const float TimeoutSeconds = 5.0f;
    if (!Future.WaitFor(FTimespan::FromSeconds(TimeoutSeconds)))
    {
        UE_LOG(LogTemp, Error, TEXT("GetRGBImageData timed out after "
                                   "%f seconds"), TimeoutSeconds);
        if (Promise.IsValid())
        {
            Promise->SetValue();  // Ensure promise is completed
        }
    }
}

RGBCameraGetOdomCall::RGBCameraGetOdomCall(
    VCCSim::RGBCameraService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, URGBCameraComponent*> rrgbcmap)
        : AsyncCallTemplateM(service, cq, rrgbcmap)
{
    Proceed(true);
}

void RGBCameraGetOdomCall::PrepareNextCall()
{
    new RGBCameraGetOdomCall(service_, cq_, RCMap_);
}

void RGBCameraGetOdomCall::InitializeRequest()
{
    service_->RequestGetRGBCameraOdom(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void RGBCameraGetOdomCall::ProcessRequest()
{
    if (RCMap_.contains(request_.name()))
    {
        const FVector Pose = RCMap_[request_.name()]->GetComponentLocation();
        const FRotator Rot = RCMap_[request_.name()]->GetComponentRotation();
        const FVector LinearVelocity = RCMap_[request_.name()]->GetPhysicsLinearVelocity();
        const FVector AngularVelocity = RCMap_[request_.name()]->GetPhysicsAngularVelocityInDegrees();

        VCCSim::Pose* PoseData = response_.mutable_pose();
        PoseData->set_x(Pose.X);
        PoseData->set_y(Pose.Y);
        PoseData->set_z(Pose.Z);
        PoseData->set_roll(Rot.Roll);
        PoseData->set_pitch(Rot.Pitch);
        PoseData->set_yaw(Rot.Yaw);

        VCCSim::twist* TwistData = response_.mutable_twist();
        TwistData->set_linear_x(LinearVelocity.X);
        TwistData->set_linear_y(LinearVelocity.Y);
        TwistData->set_linear_z(LinearVelocity.Z);
        TwistData->set_angular_x(AngularVelocity.X);
        TwistData->set_angular_y(AngularVelocity.Y);
        TwistData->set_angular_z(AngularVelocity.Z);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("RGBCameraGetOdomCall: "
                                    "RGB Camera component not found!"));
    }
}

SendMeshCall::SendMeshCall(VCCSim::MeshService::AsyncService* service,
                           grpc::ServerCompletionQueue* cq, UMeshHandlerComponent* mesh_component)
    : AsyncCallTemplate(service, cq, mesh_component)
{
    Proceed(true);
}

void SendMeshCall::PrepareNextCall()
{
    new SendMeshCall(service_, cq_, component_);
}

void SendMeshCall::InitializeRequest()
{
    service_->RequestSendMesh(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void SendMeshCall::ProcessRequest()
{
    if (component_)
    {
        FTransform MeshTransform(
            FRotator(
                request_.transform().roll(),
                request_.transform().pitch(),
                request_.transform().yaw()),
            FVector(
                request_.transform().x(),
                request_.transform().y(),
                request_.transform().z())
        );
        
        component_->UpdateMeshFromGRPC(
            reinterpret_cast<const uint8*>(request_.data().data()),
            request_.data().size(),
            MeshTransform
        );
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("SendMeshCall: Mesh component not found!"));
    }
    response_.set_status(true);
}

SendPointCloudWithColorCall::SendPointCloudWithColorCall(
    VCCSim::PointCloudService::AsyncService* service,
    grpc::ServerCompletionQueue* cq, UInsMeshHolder* mesh_holder)
    : AsyncCallTemplate(service, cq, mesh_holder)
{
    Proceed(true);
}

void SendPointCloudWithColorCall::PrepareNextCall()
{
    new SendPointCloudWithColorCall(service_, cq_, component_);
}

void SendPointCloudWithColorCall::InitializeRequest()
{
    service_->RequestSendPointCloudWithColor(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void SendPointCloudWithColorCall::ProcessRequest()
{
    if (component_)
    {
        TArray<FTransform> Transforms;
        TArray<FColor> Colors;
        for (const auto& Point : request_.data())
        {
            Transforms.Add(FTransform(
                FRotator(0, 0, 0),
                FVector(
                    Point.point().x() * 100,
                    -Point.point().y() * 100,
                    Point.point().z() * 100)
            ));
            Colors.Add(FColor(Point.color()));
        }
        component_->QueueInstanceUpdate(Transforms, Colors);
        response_.set_status(true);
    }
}

SendDronePoseCall::SendDronePoseCall(
    VCCSim::DroneService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    std::map<std::string, AQuadcopterDrone*> rcmap)
        : AsyncCallTemplateM(service, cq, rcmap)
{
    Proceed(true);
}

void SendDronePoseCall::PrepareNextCall()
{
    new SendDronePoseCall(service_, cq_, RCMap_);
}

void SendDronePoseCall::InitializeRequest()
{
    service_->RequestSendDronePose(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void SendDronePoseCall::ProcessRequest()
{
    if (RCMap_.contains(request_.name()))
    {
        if (AQuadcopterDrone* Drone = RCMap_[request_.name()])
        {
            const FVector TargetLocation(
                request_.pose().x(),
                request_.pose().y(),
                request_.pose().z()
            );
            const FRotator TargetRotation(
                request_.pose().roll(),
                request_.pose().pitch(),
                request_.pose().yaw()
            );
            if (!Drone->IfCloseToTarget(TargetLocation, TargetRotation))
            {
                Drone->SetTarget(TargetLocation, TargetRotation);
            }
            response_.set_status(true);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("SendDronePoseCall: "
                                          "AQuadcopterDrone not found!"));
            response_.set_status(false);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("SendDronePoseCall: "
                                      "Drone not found!"));
        response_.set_status(false);
    }
}