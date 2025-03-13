/*
 * Copyright (C) 2025 Visual Computing Research Center, Shenzhen University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "CoreMinimal.h"
#include <grpcpp/grpcpp.h>
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "API/VCCSim.grpc.pb.h"

class ULidarComponent;
class UDepthCameraComponent;
class URGBCameraComponent;
class UMeshHandlerComponent;
class UInsMeshHolder;
class ADronePawn;
class AFlashPawn;

class AsyncCall
{
public:
    virtual void Proceed(bool ok) = 0;
    virtual void Shutdown() = 0;
    virtual ~AsyncCall() {}
};

template <typename RequestType, typename ResponseType,
          typename ServiceType, typename ComponentType>
class AsyncCallTemplate : public AsyncCall
{
public:
    AsyncCallTemplate(ServiceType* service,
        grpc::ServerCompletionQueue* cq, ComponentType* component);

    virtual void Proceed(bool ok) override final;
    virtual void Shutdown() override final;

protected:
    virtual void PrepareNextCall() = 0;
    virtual void InitializeRequest() = 0;
    virtual void ProcessRequest() = 0;

    grpc::ServerContext ctx_;
    RequestType request_;
    ResponseType response_;
    grpc::ServerAsyncResponseWriter<ResponseType> responder_;
    
    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_;

    ServiceType* service_;
    grpc::ServerCompletionQueue* cq_;
    ComponentType* component_;
};

template <typename RequestType, typename ResponseType,
          typename ServiceType, typename RobotComponentMap>
class AsyncCallTemplateM : public AsyncCall
{
public:
    AsyncCallTemplateM(ServiceType* service,
        grpc::ServerCompletionQueue* cq, RobotComponentMap RCMap);

    virtual void Proceed(bool ok) override final;
    virtual void Shutdown() override final;

protected:
    virtual void PrepareNextCall() = 0;
    virtual void InitializeRequest() = 0;
    virtual void ProcessRequest() = 0;

    grpc::ServerContext ctx_;
    RequestType request_;
    ResponseType response_;
    grpc::ServerAsyncResponseWriter<ResponseType> responder_;
    
    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_;

    ServiceType* service_;
    grpc::ServerCompletionQueue* cq_;
    RobotComponentMap RCMap_;
};

// Image calls need to wait for the image to be captured
// So we need to let the async call to finish processing
template <typename RequestType, typename ResponseType,
          typename ServiceType, typename RobotComponentMap>
class AsyncCallTemplateImage : public AsyncCall
{
public:
    AsyncCallTemplateImage(ServiceType* service,
        grpc::ServerCompletionQueue* cq, RobotComponentMap RCMap);

    virtual void Proceed(bool ok) override final;
    virtual void Shutdown() override final;

protected:
    virtual void PrepareNextCall() = 0;
    virtual void InitializeRequest() = 0;
    virtual void ProcessRequest() = 0;

    grpc::ServerContext ctx_;
    RequestType request_;
    ResponseType response_;
    grpc::ServerAsyncResponseWriter<ResponseType> responder_;
    
    enum CallStatus { CREATE, PROCESS, FINISH };
    CallStatus status_;

    ServiceType* service_;
    grpc::ServerCompletionQueue* cq_;
    RobotComponentMap RCMap_;
};

/* ---------------------------------LiDAR---------------------------------- */
class LidarGetDataCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::LidarData,
    VCCSim::LidarService::AsyncService,
    std::map<std::string, ULidarComponent*>>
{
public:
    LidarGetDataCall(
        VCCSim::LidarService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ULidarComponent*> RLMap);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class LidarGetOdomCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::Odometry,
    VCCSim::LidarService::AsyncService,
    std::map<std::string, ULidarComponent*>>
{
public:
    LidarGetOdomCall(
        VCCSim::LidarService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ULidarComponent*> RLMap);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class LidarGetDataAndOdomCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::LidarDataAndOdom,
    VCCSim::LidarService::AsyncService,
    std::map<std::string, ULidarComponent*>>
{
public:
    LidarGetDataAndOdomCall(
        VCCSim::LidarService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ULidarComponent*> rcmap);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------Depth Camera---------------------------------- */
struct FDCPoint;
class DepthCameraGetPointDataCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::DepthCameraPointData,
    VCCSim::DepthCameraService::AsyncService,
    std::map<std::string, UDepthCameraComponent*>>
{
public:
    DepthCameraGetPointDataCall(
        VCCSim::DepthCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, UDepthCameraComponent*> rdcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
private:
    TSharedPtr<TPromise<void>> Promise;
};

class DepthCameraGetImageDataCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::DepthCameraImageData,
    VCCSim::DepthCameraService::AsyncService,
    std::map<std::string, UDepthCameraComponent*>>
{
public:
    DepthCameraGetImageDataCall(
        VCCSim::DepthCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, UDepthCameraComponent*> rdcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
private:
    TSharedPtr<TPromise<void>> Promise;
};

class DepthCameraGetOdomCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::Odometry, VCCSim::DepthCameraService::AsyncService,
    std::map<std::string, UDepthCameraComponent*>>
{
public:
    DepthCameraGetOdomCall(
        VCCSim::DepthCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, UDepthCameraComponent*> rdcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------RGB Camera---------------------------------- */

class RGBCameraGetOdomCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::Odometry, VCCSim::RGBCameraService::AsyncService,
    std::map<std::string, URGBCameraComponent*>>
{
public:
    RGBCameraGetOdomCall(
        VCCSim::RGBCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, URGBCameraComponent*> rrgbcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class RGBIndexedCameraImageDataCall : public AsyncCallTemplateImage<
    VCCSim::IndexedCamera, VCCSim::RGBCameraImageData,
    VCCSim::RGBCameraService::AsyncService,
    std::map<std::string, URGBCameraComponent*>>
{
public:
    RGBIndexedCameraImageDataCall(
        VCCSim::RGBCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, URGBCameraComponent*> rrgbcmap);

    static void InitializeImageModule()
    {
        // Load module once at startup
        if (!ImageWrapperModule)
        {
            ImageWrapperModule = &FModuleManager::LoadModuleChecked<
                IImageWrapperModule>(FName("ImageWrapper"));
        }
    }

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;

private:
    static IImageWrapperModule* ImageWrapperModule;
    TSharedPtr<IImageWrapper> ImageWrapper;
    
    // Convert LinearColor array to raw BGRA
    TArray<uint8> ConvertToBGRA(
        const TArray<FLinearColor>& ImageData, int32 Width, int32 Height)
    {
        TArray<uint8> RawBGRA;
        RawBGRA.SetNum(ImageData.Num() * 4);
        
        for (int32 i = 0; i < ImageData.Num(); i++) 
        {
            // Convert from LinearColor (which can have values outside 0-1) 
            // to standard 8-bit per channel color
            FColor SDRColor = ImageData[i].ToFColor(true); // Apply sRGB conversion
            
            // Store in BGRA order
            RawBGRA[4*i] = SDRColor.B;
            RawBGRA[4*i + 1] = SDRColor.G;
            RawBGRA[4*i + 2] = SDRColor.R;
            RawBGRA[4*i + 3] = SDRColor.A;
        }
        
        return RawBGRA;
    }
    
    // Convert to RGB raw format (uncompressed)
    TArray<uint8> ConvertToRGB(const TArray<FLinearColor>& ImageData)
    {
        TArray<uint8> RawRGB;
        RawRGB.SetNum(ImageData.Num() * 3);
        
        for (int32 i = 0; i < ImageData.Num(); i++) 
        {
            // Convert from LinearColor to standard 8-bit per channel color
            FColor SDRColor = ImageData[i].ToFColor(true);
            
            // Store in RGB order
            RawRGB[3*i] = SDRColor.R;
            RawRGB[3*i + 1] = SDRColor.G;
            RawRGB[3*i + 2] = SDRColor.B;
        }
        
        return RawRGB;
    }
    
    // Convert to compressed format (PNG or JPEG)
    TArray<uint8> ConvertToCompressedFormat(
        const TArray<FLinearColor>& ImageData, 
        int32 Width, 
        int32 Height,
        EImageFormat Format)
    {
        TArray<uint8> CompressedData;
        
        // Create appropriate image wrapper for the requested format
        TSharedPtr<IImageWrapper> FormatWrapper =
            ImageWrapperModule->CreateImageWrapper(Format);
        
        if (FormatWrapper.IsValid())
        {
            // Get BGRA data first
            TArray<uint8> RawBGRA = ConvertToBGRA(ImageData, Width, Height);
            
            if (FormatWrapper->SetRaw(RawBGRA.GetData(), RawBGRA.Num(),
                Width, Height, ERGBFormat::BGRA, 8)) 
            {
                const TArray64<uint8>& TempCompressedData = FormatWrapper->GetCompressed();
                CompressedData.Append(TempCompressedData.GetData(), TempCompressedData.Num());
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("RGBIndexedCameraImageDataCall:"
                                            " Failed to set raw image data for compression"));
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("RGBIndexedCameraImageDataCall:"
                                        " Failed to create image wrapper"));
        }
        
        return CompressedData;
    }
};

/* --------------------------Mesh Handler---------------------------------- */

class SendMeshCall : public AsyncCallTemplate<VCCSim::MeshData,
    VCCSim::Status, VCCSim::MeshService::AsyncService, UMeshHandlerComponent>
{
public:
    SendMeshCall(VCCSim::MeshService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, UMeshHandlerComponent* mesh_component);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class UFMeshManager;

class SendGlobalMeshCall : public AsyncCallTemplate<VCCSim::MeshData,
    VCCSim::MeshID, VCCSim::MeshService::AsyncService, UFMeshManager>
{
public:
    SendGlobalMeshCall(VCCSim::MeshService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, UFMeshManager* MeshManager);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class RemoveGlobalMeshCall : public AsyncCallTemplate<VCCSim::MeshID,
    VCCSim::Status, VCCSim::MeshService::AsyncService, UFMeshManager>
{
public:
    RemoveGlobalMeshCall(VCCSim::MeshService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, UFMeshManager* MeshManager);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------Misc Handler---------------------------------- */

class SendPointCloudWithColorCall : public AsyncCallTemplate<
    VCCSim::PointCloudWithColor, VCCSim::Status,
    VCCSim::PointCloudService::AsyncService, UInsMeshHolder>
{
public:
    SendPointCloudWithColorCall(VCCSim::PointCloudService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, UInsMeshHolder* mesh_holder);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------Drone Handler---------------------------------- */

class GetDronePoseCall final: public AsyncCallTemplateM<VCCSim::RobotName,
    VCCSim::Pose, VCCSim::DroneService::AsyncService,
    std::map<std::string, ADronePawn*>>
{
public:
    GetDronePoseCall(VCCSim::DroneService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ADronePawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class SendDronePoseCall : public AsyncCallTemplateM<VCCSim::DronePose,
    VCCSim::Status, VCCSim::DroneService::AsyncService,
    std::map <std::string, ADronePawn*>>
{
public:
    SendDronePoseCall(VCCSim::DroneService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ADronePawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class SendDronePathCall : public AsyncCallTemplateM<VCCSim::DronePath,
    VCCSim::Status, VCCSim::DroneService::AsyncService,
    std::map<std::string, ADronePawn*>>
{
public:
    SendDronePathCall(VCCSim::DroneService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, ADronePawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------Flash Handler--------------------------------- */

class GetFlashPoseCall final: public AsyncCallTemplateM<VCCSim::RobotName,
    VCCSim::Pose, VCCSim::FlashService::AsyncService,
    std::map<std::string, AFlashPawn*>>
{
public:
    GetFlashPoseCall(VCCSim::FlashService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AFlashPawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class SendFlashPoseCall : public AsyncCallTemplateM<VCCSim::FlashPose,
    VCCSim::Status, VCCSim::FlashService::AsyncService,
    std::map<std::string, AFlashPawn*>>
{
public:
    SendFlashPoseCall(VCCSim::FlashService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AFlashPawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class SendFlashPathCall : public AsyncCallTemplateM<VCCSim::FlashPath,
    VCCSim::Status, VCCSim::FlashService::AsyncService,
    std::map<std::string, AFlashPawn*>>
{
public:
    SendFlashPathCall(VCCSim::FlashService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AFlashPawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class CheckFlashReadyCall : public AsyncCallTemplateM<VCCSim::RobotName,
    VCCSim::Status, VCCSim::FlashService::AsyncService,
    std::map<std::string, AFlashPawn*>>
{
public:
    CheckFlashReadyCall(VCCSim::FlashService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AFlashPawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class MoveToNextCall : public AsyncCallTemplateM<VCCSim::RobotName,
    VCCSim::Status, VCCSim::FlashService::AsyncService,
    std::map<std::string, AFlashPawn*>>
{
public:
    MoveToNextCall(VCCSim::FlashService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AFlashPawn*> rcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

/* --------------------------Template implementation----------------------- */
    
template <typename RequestType, typename ResponseType,
          typename ServiceType, typename ComponentType>
AsyncCallTemplate<RequestType, ResponseType,
    ServiceType, ComponentType>::AsyncCallTemplate(
    ServiceType* service, grpc::ServerCompletionQueue* cq, ComponentType* component)
    : responder_(&ctx_), status_(CREATE), service_(service),
      cq_(cq), component_(component) {}

template <typename RequestType, typename ResponseType,
          typename ServiceType, typename ComponentType>
void AsyncCallTemplate<RequestType, ResponseType, ServiceType,
    ComponentType>::Proceed(bool ok)
{
    if (status_ == CREATE) {
        status_ = PROCESS;
        InitializeRequest();
    }
    else if (status_ == PROCESS) {
        PrepareNextCall();
        ProcessRequest();

        status_ = FINISH;
        responder_.Finish(response_, grpc::Status::OK, this);
    }
    else {
        delete this;
    }
}

template <typename RequestType, typename ResponseType,
          typename ServiceType, typename ComponentType>
void AsyncCallTemplate<RequestType, ResponseType,
    ServiceType, ComponentType>::Shutdown()
{
    status_ = FINISH;
    delete this;
}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
AsyncCallTemplateM<RequestType, ResponseType, ServiceType, RobotComponentMap>::
AsyncCallTemplateM(ServiceType* service, grpc::ServerCompletionQueue* cq,
    RobotComponentMap RCMap)
    : responder_(&ctx_), status_(CREATE), service_(service),
      cq_(cq), RCMap_(RCMap) {}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
void AsyncCallTemplateM<RequestType, ResponseType, ServiceType,
RobotComponentMap>::Proceed(bool ok)
{
    if (status_ == CREATE) {
        status_ = PROCESS;
        InitializeRequest();
    }
    else if (status_ == PROCESS) {
        PrepareNextCall();
        ProcessRequest();

        status_ = FINISH;
        responder_.Finish(response_, grpc::Status::OK, this);
    }
    else {
        delete this;
    }
}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
void AsyncCallTemplateM<RequestType, ResponseType, ServiceType,
RobotComponentMap>::Shutdown()
{
    status_ = FINISH;
    delete this;
}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
AsyncCallTemplateImage<RequestType, ResponseType, ServiceType, RobotComponentMap>::
AsyncCallTemplateImage(ServiceType* service, grpc::ServerCompletionQueue* cq,
    RobotComponentMap RCMap)
    : responder_(&ctx_), status_(CREATE), service_(service),
      cq_(cq), RCMap_(RCMap) {}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
void AsyncCallTemplateImage<RequestType, ResponseType, ServiceType,
RobotComponentMap>::Proceed(bool ok)
{
    if (status_ == CREATE) {
        status_ = PROCESS;
        InitializeRequest();
    }
    else if (status_ == PROCESS) {
        PrepareNextCall();
        ProcessRequest();
    }
    else {
        delete this;
    }
}

template <typename RequestType, typename ResponseType, typename ServiceType,
    typename RobotComponentMap>
void AsyncCallTemplateImage<RequestType, ResponseType, ServiceType,
RobotComponentMap>::Shutdown()
{
    status_ = FINISH;
    delete this;
}
