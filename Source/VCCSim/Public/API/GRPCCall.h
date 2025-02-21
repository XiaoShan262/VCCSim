// MIT License
// 
// Copyright (c) 2025 Mingyang Wang
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
            ImageWrapperModule = &FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
        }
    }
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
private:
    static IImageWrapperModule* ImageWrapperModule;
    TSharedPtr<IImageWrapper> ImageWrapper;
    
    TArray<uint8> ConvertToPNG(const TArray<FColor>& ImageData, int32 Width, int32 Height) 
    {
        TArray<uint8> PNGData;
        
        if (ImageWrapper.IsValid())
        {
            // Convert FColor array to raw BGRA
            TArray<uint8> RawBGRA;
            RawBGRA.SetNum(ImageData.Num() * 4);
            for (int32 i = 0; i < ImageData.Num(); i++) 
            {
                RawBGRA[4*i] = ImageData[i].B;
                RawBGRA[4*i + 1] = ImageData[i].G;
                RawBGRA[4*i + 2] = ImageData[i].R;
                RawBGRA[4*i + 3] = ImageData[i].A;
            }
            
            if (ImageWrapper->SetRaw(RawBGRA.GetData(), RawBGRA.Num(),
                Width, Height, ERGBFormat::BGRA, 8)) 
            {
                const TArray64<uint8>& CompressedData = ImageWrapper->GetCompressed();
                PNGData.Append(CompressedData.GetData(), CompressedData.Num());
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("RGBIndexedCameraImageDataCall:"
                                            "Failed to set raw image data for PNG conversion"));
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("RGBIndexedCameraImageDataCall:"
                                        "Failed to create PNG image wrapper"));
        }
        
        return PNGData;
    }
};

    
/* --------------------------Misc Handler---------------------------------- */
class SendMeshCall : public AsyncCallTemplate<VCCSim::MeshData,
    VCCSim::Status, VCCSim::MeshService::AsyncService, UMeshHandlerComponent> {
public:
    SendMeshCall(VCCSim::MeshService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, UMeshHandlerComponent* mesh_component);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

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

/* -----------------------Template implementation----------------------- */
    
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
