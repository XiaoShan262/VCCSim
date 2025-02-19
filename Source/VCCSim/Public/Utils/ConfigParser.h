#pragma once
#include <string>
#include <vector>
#include <set>
#include <memory>
#include "Sensors/SensorBase.h"
#include "Pawns/PawnBase.h"

using FComponentConfig = std::pair<ESensorType, std::shared_ptr<SensorConfig>>;

struct FRobot
{
	std::string UETag;
	EPawnType Type;
	std::vector<FComponentConfig> ComponentConfigs;
	std::set<ESensorType> RecordComponents;
};

struct FVCCSimPresets
{
	std::string Server;
	std::string MainCharacter;
	std::vector<std::string> StaticMeshActor;
	std::vector<std::string> SubWindows{};
	std::vector<float> SubWindowsOpacities;
	int LS_StartOffset;
	std::string LogSavePath;
	std::string DefaultDronePawn;
	std::string DefaultCarPawn;
	float RecordInterval;
	int BufferSize;
};

struct FVCCSimConfig
{
	FVCCSimPresets VCCSim;
	std::vector<FRobot> Robots;
};

FVCCSimConfig ParseConfig();