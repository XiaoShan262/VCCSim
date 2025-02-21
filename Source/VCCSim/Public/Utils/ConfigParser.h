#pragma once
#include <string>
#include <vector>
#include <set>
#include <memory>
#include "Sensors/SensorBase.h"
#include "Pawns/PawnBase.h"

using FComponentConfig = std::pair<ESensorType, std::shared_ptr<FSensorConfig>>;

struct FRobot
{
	std::string UETag;
	EPawnType Type;
	std::vector<FComponentConfig> ComponentConfigs;
	float RecordInterval;
};

struct FVCCSimPresets
{
	std::string Server;
	std::string MainCharacter;
	std::vector<std::string> StaticMeshActor;
	std::vector<std::string> SubWindows{};
	std::vector<float> SubWindowsOpacities;
	int LS_StartOffset;
	bool StartWithRecording;
	std::string LogSavePath;
	std::string DefaultDronePawn;
	std::string DefaultCarPawn;
	int BufferSize;
};

struct FVCCSimConfig
{
	FVCCSimPresets VCCSim;
	std::vector<FRobot> Robots;
};

FVCCSimConfig ParseConfig();