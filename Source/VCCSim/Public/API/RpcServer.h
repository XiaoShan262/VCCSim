#ifndef LIDAR_SERVICE_SERVER_H
#define LIDAR_SERVICE_SERVER_H

#include <map>
#include <string>

class FGrpcServerTask;
class ULidarComponent;
class UDepthCameraComponent;
class URGBCameraComponent;
struct FVCCSimConfig;

struct FRobotGrpcMaps
{
	struct FRobotComponentMaps
	{
		std::map<std::string, ULidarComponent*> RLMap;
		std::map<std::string, UDepthCameraComponent*> RDCMap;
		std::map<std::string, URGBCameraComponent*> RRGBCMap;
	};

	struct FRobotMaps
	{
		std::map<std::string, AActor*> DroneMap;
		std::map<std::string, AActor*> CarMap;
	};

	FRobotComponentMaps RCMaps;
	FRobotMaps RMaps;
};


extern FAsyncTask<FGrpcServerTask>* Server_Task;

void RunServer(const FVCCSimConfig& Config, AActor* Holder,
	const FRobotGrpcMaps& RGrpcMaps);

void ShutdownServer();

#endif // LIDAR_SERVICE_SERVER_H
