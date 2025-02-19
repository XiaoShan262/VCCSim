#include "API/RpcServer.h"
#include "API/GRPCCall.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Pawns/DronePawn.h"
#include "Utils/MeshHandlerComponent.h"
#include "Async/AsyncWork.h"
#include "Utils/ConfigParser.h"
#include <iostream>
#include <memory>


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;


FAsyncTask<FGrpcServerTask>* Server_Task = nullptr;
std::atomic<bool> ShutdownRequested = false;


class FGrpcServerTask : public FNonAbandonableTask
{
public:
    FGrpcServerTask(
    	const std::string& InServerAddress,
    	UMeshHandlerComponent* MeshComponent, UInsMeshHolder* InstancedMeshHolder,
    	FRobotGrpcMaps RobotGrpcMap)
        : ServerAddress(InServerAddress),
		  MeshComponent(MeshComponent), InstancedMeshHolder(InstancedMeshHolder),
		  RGrpcMaps(RobotGrpcMap) {}

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FGrpcServerTask, STATGROUP_ThreadPoolAsyncTasks);
    }

    void DoWork()
    {
    	VCCSim::DroneService::AsyncService DroneService;
    	VCCSim::LidarService::AsyncService LidarService;
    	VCCSim::DepthCameraService::AsyncService DepthCameraService;
    	VCCSim::RGBCameraService::AsyncService RGBCameraService;
    	VCCSim::MeshService::AsyncService MeshService;
    	VCCSim::PointCloudService::AsyncService PointCloudService;
    	
        grpc::ServerBuilder Builder;
        Builder.AddListeningPort(ServerAddress, grpc::InsecureServerCredentials());
    	Builder.RegisterService(&DroneService);
    	Builder.RegisterService(&DepthCameraService);
    	Builder.RegisterService(&RGBCameraService);
        Builder.RegisterService(&LidarService);
        Builder.RegisterService(&MeshService);
    	Builder.RegisterService(&PointCloudService);
    	
        CompletionQueue = Builder.AddCompletionQueue();
        Server = Builder.BuildAndStart();

        if (Server)
        {
            UE_LOG(LogTemp, Warning, TEXT("Asynchronous Server listening on %s"),
            	*FString(ServerAddress.c_str()));

            // Spawn initial asynchronous calls
        	std::map<std::string, AQuadcopterDrone*> DroneMap;
        	for (const auto& Pair : RGrpcMaps.RMaps.DroneMap)
			{
				DroneMap[Pair.first] = Cast<AQuadcopterDrone>(Pair.second);
			}
        	new SendDronePoseCall(&DroneService, CompletionQueue.get(), DroneMap);
        	
        	// new LidarGetDataCall(&service, CompletionQueue.get(), LidarComponent);
        	// new LidarGetOdomCall(&service, CompletionQueue.get(), LidarComponent);
        	new LidarGetDataAndOdomCall(&LidarService,
        		CompletionQueue.get(), RGrpcMaps.RCMaps.RLMap);
        	
			new DepthCameraGetImageDataCall(&DepthCameraService,
				CompletionQueue.get(), RGrpcMaps.RCMaps.RDCMap);
        	new DepthCameraGetPointDataCall(&DepthCameraService,
				CompletionQueue.get(), RGrpcMaps.RCMaps.RDCMap);
			new DepthCameraGetOdomCall(&DepthCameraService,
				CompletionQueue.get(), RGrpcMaps.RCMaps.RDCMap);

        	new RGBCameraGetOdomCall(&RGBCameraService,
        		CompletionQueue.get(), RGrpcMaps.RCMaps.RRGBCMap);
        	new RGBIndexedCameraImageDataCall(&RGBCameraService,
				CompletionQueue.get(), RGrpcMaps.RCMaps.RRGBCMap);
        	
        	new SendMeshCall(&MeshService, CompletionQueue.get(), MeshComponent);
        	new SendPointCloudWithColorCall(&PointCloudService,
        		CompletionQueue.get(), InstancedMeshHolder);

            // Start the completion queue processing loop
            CompletionQueueThread = std::thread([this]() {
                void* tag;
                bool ok;
                while (CompletionQueue->Next(&tag, &ok))
                {
                	AsyncCall* call = static_cast<AsyncCall*>(tag);
                	
                	if (ShutdownRequested.load())
                	{
                		call->Shutdown();
                	}
	                else
	                {
	                	call->Proceed(ok);
	                }
                }
            });

            // Wait until shutdown is requested
            while (!ShutdownRequested.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

        	// Real shutdown
        	UE_LOG(LogTemp, Warning, TEXT("DoWork() sees ShutdownRequested,"
									   " shutting down server..."));

        	if (Server)
        	{
        		Server->Shutdown();
        	}
        	if (CompletionQueue)
        	{
        		CompletionQueue->Shutdown();
        	}
        	if (CompletionQueueThread.joinable())
        	{
        		CompletionQueueThread.join();
        	}
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to start server on %s"),
            	*FString(ServerAddress.c_str()));
        }
    }

private:
    std::string ServerAddress;
	UMeshHandlerComponent* MeshComponent = nullptr;
	UInsMeshHolder* InstancedMeshHolder = nullptr;
    FRobotGrpcMaps RGrpcMaps;
	
    std::unique_ptr<grpc::ServerCompletionQueue> CompletionQueue;
    std::unique_ptr<grpc::Server> Server;
    std::thread CompletionQueueThread;
};

void RunServer(const FVCCSimConfig& Config, AActor* Holder,
	const FRobotGrpcMaps& RGrpcMaps)
{
	if (ShutdownRequested.load())
	{
		UE_LOG(LogTemp, Warning, TEXT("Server is already running!"
							  "Not Cleaning up before starting a new one."));
		ShutdownRequested.store(false);
	}
	
	Server_Task = new FAsyncTask<FGrpcServerTask>(
		Config.VCCSim.Server,
		Holder->FindComponentByClass<UMeshHandlerComponent>(),
		Holder->FindComponentByClass<UInsMeshHolder>(),
		RGrpcMaps);
	Server_Task->StartBackgroundTask();
	UE_LOG(LogTemp, Warning, TEXT("GRPC server started."));
}

void ShutdownServer()
{
	if (Server_Task)
	{		
		ShutdownRequested.store(true);
		Server_Task->EnsureCompletion();
		
		delete Server_Task;
		Server_Task = nullptr;
		
		UE_LOG(LogTemp, Warning, TEXT("GRPC Server shutdown."));
	}
	ShutdownRequested.store(false);
}