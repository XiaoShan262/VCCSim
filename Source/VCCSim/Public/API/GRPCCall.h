#pragma once

#include "CoreMinimal.h"
#include <grpcpp/grpcpp.h>
#include "API/VCCSim.grpc.pb.h"

class ULidarComponent;
class UDepthCameraComponent;
class URGBCameraComponent;
class UMeshHandlerComponent;
class UInsMeshHolder;
class AQuadcopterDrone;

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

/* ---------------------------------LiDAR---------------------------------- */
class LidarGetDataCall : public AsyncCallTemplate<VCCSim::RobotName,
    VCCSim::LidarData, VCCSim::LidarService::AsyncService, ULidarComponent> {
public:
    LidarGetDataCall(VCCSim::LidarService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, ULidarComponent* lidar_component);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class LidarGetOdomCall : public AsyncCallTemplate<VCCSim::RobotName,
    VCCSim::Odometry, VCCSim::LidarService::AsyncService, ULidarComponent> {
public:
    LidarGetOdomCall(VCCSim::LidarService::AsyncService* service,
        grpc::ServerCompletionQueue* cq, ULidarComponent* lidar_component);

protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
};

class LidarGetDataAndOdomCall : public AsyncCallTemplateM<
    VCCSim::RobotName, VCCSim::LidarDataAndOdom, VCCSim::LidarService::AsyncService,
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

class RGBIndexedCameraImageDataCall : public AsyncCallTemplateM<
    VCCSim::IndexedCamera, VCCSim::RGBCameraImageData,
    VCCSim::RGBCameraService::AsyncService,
    std::map<std::string, URGBCameraComponent*>>
{
public:
    RGBIndexedCameraImageDataCall(
        VCCSim::RGBCameraService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, URGBCameraComponent*> rrgbcmap);
protected:
    virtual void PrepareNextCall() override final;
    virtual void InitializeRequest() override final;
    virtual void ProcessRequest() override final;
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

class SendDronePoseCall : public AsyncCallTemplateM<VCCSim::DronePose,
    VCCSim::Status, VCCSim::DroneService::AsyncService,
    std::map <std::string, AQuadcopterDrone*>>
{
public:
    SendDronePoseCall(VCCSim::DroneService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        std::map<std::string, AQuadcopterDrone*> rcmap);
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

