#include "Utils/ConfigParser.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/CameraSensor.h"
#include "toml++/toml.hpp"
#include <filesystem>
#include "CoreMinimal.h"

using namespace std::literals;


FVCCSimConfig ParseConfig()
{
    FVCCSimConfig Config;
    
    std::string Filename = TCHAR_TO_UTF8(*FPaths::Combine(FPaths::ProjectPluginsDir(),
        TEXT("VCCSim/Source/VCCSim/RSConfig.toml")));;
    
    if (!std::filesystem::exists(Filename))
    {
        UE_LOG(LogTemp, Warning, TEXT("ParseConfig: Using default config!"));
        // Get the user's documents directory
        FString DocPath = FPlatformProcess::UserDir();
        Filename = TCHAR_TO_UTF8(*FPaths::Combine(DocPath, TEXT("VCCSim"), TEXT("RSConfig.toml")));
    }

    auto Tbl = toml::parse_file(Filename);

    if (auto VCCSim = Tbl["VCCSimPresets"].as_table())
    {
        Config.VCCSim.Server =
            std::string((*VCCSim)["IP"].value_or("0.0.0.0"sv)) + ":" +
            std::to_string((*VCCSim)["Port"].value_or(50996));
        Config.VCCSim.MainCharacter = (*VCCSim)["MainCharacter"].value_or("");
        Config.VCCSim.LS_StartOffset = (*VCCSim)["LS_StartOffset"].value_or(0);
        Config.VCCSim.LogSavePath = (*VCCSim)["LogSavePath"].value_or(
            TCHAR_TO_UTF8(FPlatformProcess::UserDir()));
        Config.VCCSim.DefaultDronePawn = (*VCCSim)["DefaultDronePawn"].value_or("");
        Config.VCCSim.DefaultCarPawn = (*VCCSim)["DefaultCarPawn"].value_or("");
        
        if (auto staticMeshActors = (*VCCSim)["StaticMeshActor"].as_array())
        {
            for (const auto& actor : *staticMeshActors)
            {
                if (auto actorName = actor.value<std::string>())
                {
                    Config.VCCSim.StaticMeshActor.push_back(*actorName);
                }
            }
        }
        if (auto subWindows = (*VCCSim)["SubWindows"].as_array())
        {
            for (const auto& window : *subWindows)
            {
                if (auto windowName = window.value<std::string>())
                {
                    Config.VCCSim.SubWindows.push_back(*windowName);
                }
            }
        }
        if (auto subWindowsOpacities
            = (*VCCSim)["SubWindowsOpacities"].as_array())
        {
            for (const auto& opacity : *subWindowsOpacities)
            {
                if (auto op = opacity.value<double>())
                {
                    Config.VCCSim.SubWindowsOpacities.push_back(*op);
                }
            }
        }
    }

    if (auto robots = Tbl["Robots"].as_array())
    {
        for (const auto& robot : *robots)
        {
            FRobot r;
            auto robotDetails = *robot.as_table();

            r.UETag = robotDetails["UETag"].value_or("None"sv);
            if (r.UETag == "None")
            {
                UE_LOG(LogTemp, Error, TEXT("ParseConfig: Robot name not found!"));
                continue;
            }

            r.Type = robotDetails["Type"].value_or("None"sv) == "Drone" ?
                EPawnType::Drone : EPawnType::Car;

            if (auto ComponentConfigs = robotDetails["ComponentConfigs"].as_table())
            {
                for (const auto& [comp_name, comp_config] : *ComponentConfigs)
                {
                    if (comp_name == "Lidar")
                    {
                        auto lidarConfig = std::make_shared<LiDARConfig>();
                        if (auto table = comp_config.as_table())
                        {
                            lidarConfig->NumRays =
                                (*table)["NumRays"].value_or(lidarConfig->NumRays);
                            lidarConfig->NumPoints =
                                (*table)["NumPoints"].value_or(lidarConfig->NumPoints);
                            lidarConfig->ScannerRangeInner =
                                (*table)["ScannerRangeInner"].value_or(lidarConfig->ScannerRangeInner);
                            lidarConfig->ScannerRangeOuter =
                                (*table)["ScannerRangeOuter"].value_or(lidarConfig->ScannerRangeOuter);
                            lidarConfig->ScannerAngleUp =
                                (*table)["ScannerAngle"].value_or(lidarConfig->ScannerAngleUp);
                            lidarConfig->ScannerAngleDown =
                                (*table)["ScannerAngleDown"].value_or(lidarConfig->ScannerAngleDown);
                            lidarConfig->bVisualizePoints =
                                (*table)["bVisualizePoints"].value_or(lidarConfig->bVisualizePoints);
                        }
                        r.ComponentConfigs.push_back({ESensorType::Lidar, lidarConfig});
                    }
                    else if (comp_name == "DepthCamera")
                    {
                        auto depthConfig = std::make_shared<DepthCameraConfig>();
                        if (auto table = comp_config.as_table())
                        {
                            depthConfig->FOV = (*table)["FOV"].value_or(depthConfig->FOV);
                            depthConfig->MaxRange = (*table)["MaxRange"].value_or(depthConfig->MaxRange);
                            depthConfig->MinRange = (*table)["MinRange"].value_or(depthConfig->MinRange);
                            depthConfig->Width = (*table)["Width"].value_or(depthConfig->Width);
                            depthConfig->Height = (*table)["Height"].value_or(depthConfig->Height);
                            depthConfig->bOrthographic =
                                (*table)["bOrthographic"].value_or(depthConfig->bOrthographic);
                            depthConfig->OrthoWidth =
                                (*table)["OrthoWidth"].value_or(depthConfig->OrthoWidth);
                            depthConfig->CaptureRate =
                                (*table)["CaptureRate"].value_or(depthConfig->CaptureRate);
                        }
                        r.ComponentConfigs.push_back({ESensorType::DepthCamera, depthConfig});
                    }
                    else if (comp_name == "RGBCamera")
                    {
                        auto rgbConfig = std::make_shared<RGBCameraConfig>();
                        if (auto table = comp_config.as_table())
                        {
                            rgbConfig->FOV = (*table)["FOV"].value_or(rgbConfig->FOV);
                            rgbConfig->Width = (*table)["Width"].value_or(rgbConfig->Width);
                            rgbConfig->Height = (*table)["Height"].value_or(rgbConfig->Height);
                            rgbConfig->bOrthographic =
                                (*table)["bOrthographic"].value_or(rgbConfig->bOrthographic);
                            rgbConfig->OrthoWidth =
                                (*table)["OrthoWidth"].value_or(rgbConfig->OrthoWidth);
                            rgbConfig->CaptureRate =
                                (*table)["CaptureRate"].value_or(rgbConfig->CaptureRate);
                        }
                        r.ComponentConfigs.push_back({ESensorType::RGBCamera, rgbConfig});
                    }
                    else
                    {
                        UE_LOG(LogTemp, Warning, TEXT("ParseConfig: Component %s not found!"),
                            *FString{UTF8_TO_TCHAR(std::string(comp_name).c_str())});
                    }
                }
            }

            Config.Robots.push_back(std::move(r));
        }
    }

    return Config;
}