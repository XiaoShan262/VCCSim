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

#include "Core/VCCHUD.h"
#include "Core/MenuWidgets.h"
#include "API/RpcServer.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/CameraSensor.h"
#include "Simulation/Recorder.h"
#include "Simulation/MeshManager.h"
#include "Utils/ConfigParser.h"
#include "Utils/InsMeshHolder.h"
#include "Utils/VCCSIMDisplayWidget.h"
#include "EnhancedInputComponent.h"
#include "LevelSequencePlayer.h"
#include "LevelSequenceActor.h"
#include "Kismet/GameplayStatics.h"


void AVCCHUD::BeginPlay()
{
    Super::BeginPlay();
    
    const FVCCSimConfig Config = ParseConfig();
    SetupRecorder(Config);
    SetupWidgetsAndLS(Config);
    auto RCMaps = SetupActors(Config);
    RunServer(Config, Holder, RCMaps);
}

void AVCCHUD::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);

    if (APlayerController* PC = GetOwningPlayerController())
    {
        // Clean up Enhanced Input Subsystem
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
            ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
        {
            Subsystem->RemoveMappingContext(DefaultMappingContext);
        }

        // Clean up InputComponent bindings
        if (UEnhancedInputComponent* EnhancedInputComponent = 
            Cast<UEnhancedInputComponent>(PC->InputComponent))
        {
            EnhancedInputComponent->ClearBindingsForObject(this);
        }
    }

    if (CurrentPauseMenu)
    {
        CurrentPauseMenu->RemoveFromParent();
    }

    if (WidgetInstance)
    {
        WidgetInstance->RemoveFromParent();
    }
    
    ShutdownServer();
}

void AVCCHUD::SetupEnhancedInput()
{
    if (!DefaultMappingContext || !PauseAction) return;
    APlayerController* PC = GetOwningPlayerController();
    if (!PC) return;

    // Get the local player subsystem
    if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
        ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
    {
        Subsystem->AddMappingContext(DefaultMappingContext, 1);
    }

    if (UEnhancedInputComponent* EnhancedInputComponent =
        Cast<UEnhancedInputComponent>(PC->InputComponent))
    {
        // Bind the pause action
        EnhancedInputComponent->BindAction(PauseAction, ETriggerEvent::Triggered,
            this, &AVCCHUD::OnPauseActionTriggered);
        if (ToggleRecordingAction)
        {
            EnhancedInputComponent->BindAction(ToggleRecordingAction, ETriggerEvent::Started,
                this, &AVCCHUD::OnToggleRecordingTriggered);
        }
    }

    PC->SetInputMode(FInputModeGameOnly());
    PC->bShowMouseCursor = false;
}

void AVCCHUD::OnPauseActionTriggered()
{    
    if (CurrentPauseMenu)
    {
        CurrentPauseMenu->RemoveFromParent();
        CurrentPauseMenu = nullptr;
    }

    if (PauseWidgetClass)
    {
        CurrentPauseMenu = CreateWidget<UPauseMenuWidget>(
            GetOwningPlayerController(), PauseWidgetClass);
        CurrentPauseMenu->AddToViewport();
                
        if (APlayerController* PC = GetOwningPlayerController())
        {
            PC->SetInputMode(FInputModeUIOnly());
            PC->SetShowMouseCursor(true);
            UGameplayStatics::SetGamePaused(GetWorld(), true);
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PauseWidgetClass not set"));
    }
}

void AVCCHUD::OnToggleRecordingTriggered()
{
    if (Recorder)
    {
        Recorder->ToggleRecording();
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Recorder not found!"));
    }
}

void AVCCHUD::SetupRecorder(const FVCCSimConfig& Config)
{
    Recorder = GetWorld()->SpawnActor<ARecorder>(ARecorder::StaticClass(), FTransform::Identity);
    Recorder->LogBasePath = Config.VCCSim.LogSavePath.c_str();
    Recorder->BufferSize = Config.VCCSim.BufferSize;
    Recorder->RecordState = Config.VCCSim.StartWithRecording;
    Recorder->StartRecording();
}

void AVCCHUD::SetupWidgetsAndLS(const FVCCSimConfig& Config)
{
    if (!WidgetClass)
    {
        static ConstructorHelpers::FClassFinder<UVCCSIMDisplayWidget>
            HUDWidgetClass(TEXT("WidgetBlueprint'/VCCSim/HUD/BP_VCCSIMDisplayWidget_VF'"));
        WidgetClass = HUDWidgetClass.Succeeded() ? HUDWidgetClass.Class : nullptr;
    }

    WidgetInstance = CreateWidget<UVCCSIMDisplayWidget>(GetWorld(), WidgetClass);
    if (WidgetInstance)
    {
        WidgetInstance->AddToViewport();
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create widget instance"));
    }

    SetupEnhancedInput();
    
    WidgetInstance->LogSavePath = Config.VCCSim.LogSavePath.c_str();

    TArray<AActor*> LevelSequenceActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ALevelSequenceActor::StaticClass(), 
                                          LevelSequenceActors);

    for (AActor* Actor : LevelSequenceActors)
    {
        ALevelSequenceActor* LevelSequenceActor = Cast<ALevelSequenceActor>(Actor);
        if (LevelSequenceActor && LevelSequenceActor->ActorHasTag(FName(TEXT("MainShowOff"))))
        {
            if (ULevelSequencePlayer* SequencePlayer = LevelSequenceActor->GetSequencePlayer())
            {
                FFrameTime NewStartTime = Config.VCCSim.LS_StartOffset;
                FMovieSceneSequencePlaybackParams PlaybackParams(NewStartTime,
                    EUpdatePositionMethod::Play);
                SequencePlayer->SetPlaybackPosition(PlaybackParams);
            }
            break;
        }
    }

    Holder = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity);
    WidgetInstance->SetHolder(Holder);
    WidgetInstance->InitFromConfig(Config);
}

void AVCCHUD::SetupMainCharacter(const FVCCSimConfig& Config, TArray<AActor*> FoundPawns)
{
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), APawn::StaticClass(), FoundPawns);
    
    for (AActor* Actor : FoundPawns)
    {
        if (Actor->ActorHasTag(Config.VCCSim.MainCharacter.c_str()))
        {
            MainCharacter = Cast<APawn>(Actor);
            break;
        }
    }
    if (!MainCharacter)
    {
        for (AActor* Actor : FoundPawns)
        {
            if (Actor->GetName().Contains(Config.VCCSim.MainCharacter.c_str()))
            {
                MainCharacter = Cast<APawn>(Actor);
                break;
            }
        }
    }
    
    if (!MainCharacter)
    {
        UE_LOG(LogTemp, Warning, TEXT("AVCCHUD::SetupMainCharacter: "
                                      "MainCharacter not found!"));
        return;
    }

    // Set the camera as the view target
    APlayerController* PlayerController = GetWorld()->GetFirstPlayerController();
    if (PlayerController)
    {
        PlayerController->SetViewTarget(MainCharacter);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("PlayerController not found!"));
    }

    if (PlayerController && MainCharacter)
    {
        PlayerController->Possess(MainCharacter);
        if (auto Func = MainCharacter->FindFunction(FName(TEXT("AddMapContext"))))
        {
            MainCharacter->ProcessEvent(Func, nullptr);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("AddMapContext not found!"));
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to possess MainCharacter!"));
    }

    FRobot MainRobotConfig;
    for (const FRobot& Robot : Config.Robots)
    {
        if (Robot.UETag == Config.VCCSim.MainCharacter)
        {
            MainRobotConfig = Robot;
            break;
        }
    }
    for (const auto& Component : MainRobotConfig.ComponentConfigs)
    {
        if (Component.first == ESensorType::DepthCamera)
        {
            if (UDepthCameraComponent* DepthCameraComponent =
                MainCharacter->FindComponentByClass<UDepthCameraComponent>())
            {
                WidgetInstance->SetDepthTexture(DepthCameraComponent->DepthRenderTarget);               
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("AVCCHUD: "
                                              "DepthCamera component not found!"));
            }
        }
        if (Component.first == ESensorType::RGBCamera)
        {
            if (URGBCameraComponent* RGBCameraComponent =
                MainCharacter->FindComponentByClass<URGBCameraComponent>())
            {
                WidgetInstance->SetRGBTexture(RGBCameraComponent->RGBRenderTarget);
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("AVCCHUD: "
                                              "RGBCamera component not found!"));
            }
        }
    }
}

FRobotGrpcMaps AVCCHUD::SetupActors(const FVCCSimConfig& Config)
{
    TArray<AActor*> FoundPawns;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), APawn::StaticClass(), FoundPawns);

    FRobotGrpcMaps RGrpcMaps;
    
    for (const FRobot& Robot : Config.Robots)
    {
        APawn* RobotPawn = nullptr;
        for (AActor* Actor : FoundPawns)
        {
            if (Actor->ActorHasTag(Robot.UETag.c_str()))
            {
                RobotPawn = Cast<APawn>(Actor);
                break;
            }
        }
        if (!RobotPawn)
        {
            for (AActor* Actor : FoundPawns)
            {
                if (Actor->GetName().Contains(Robot.UETag.c_str()))
                {
                    RobotPawn = Cast<APawn>(Actor);
                    break;
                }
            }
        }
        if (!RobotPawn)
        {
            UE_LOG(LogTemp, Warning, TEXT("Robot %s not found! Creating a new one"),
                *FString(UTF8_TO_TCHAR(Robot.UETag.c_str())));
            RobotPawn = CreatePawn(Config, Robot);
            if (!RobotPawn)
            {
                UE_LOG(LogTemp, Error, TEXT("Failed to create Robot %s"),
                    *FString(UTF8_TO_TCHAR(Robot.UETag.c_str())));
                continue;
            }
        }
        
        if (Robot.Type == EPawnType::Drone)
        {
            RGrpcMaps.RMaps.DroneMap[Robot.UETag] = RobotPawn;
        }
        else if (Robot.Type == EPawnType::Car)
        {
            RGrpcMaps.RMaps.CarMap[Robot.UETag] = RobotPawn;
        }
        else if (Robot.Type == EPawnType::Flash)
        {
            RGrpcMaps.RMaps.FlashMap[Robot.UETag] = RobotPawn;
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("AVCCHUD::SetupActors:"
                                        "Unknown pawn type!"));
        }

        if (Robot.RecordInterval > 0)
        {
            if (auto Func = RobotPawn->FindFunction(FName(TEXT("SetRecorder"))))
            {
                RobotPawn->ProcessEvent(Func, &Recorder);
            }
            if (auto Func = RobotPawn->FindFunction(FName(TEXT("SetRecordInterval"))))
            {
                auto RecordInterval = Robot.RecordInterval;
                RobotPawn->ProcessEvent(Func, &RecordInterval);
            }
        }

        bool bHasLidar = false;
        bool bHasDepth = false;
        bool bHasRGB = false;
        
        for (const auto& Component : Robot.ComponentConfigs)
        {
            if (Component.first == ESensorType::Lidar)
            {
                ULidarComponent* LidarComponent =
                    RobotPawn->FindComponentByClass<ULidarComponent>();
                LidarComponent->RConfigure(
                    *static_cast<FLiDarConfig*>(Component.second.get()),
                    Recorder);
                LidarComponent->FirstCall();
                LidarComponent->MeshHolder = Holder->FindComponentByClass<UInsMeshHolder>();
                RGrpcMaps.RCMaps.RLMap[Robot.UETag] = LidarComponent;

                if (LidarComponent->bRecorded)
                {
                    bHasLidar = true;
                }
            }
            else if (Component.first == ESensorType::DepthCamera)
            {
                TArray<UDepthCameraComponent*> DepthCameras;
                RobotPawn->GetComponents<UDepthCameraComponent>(DepthCameras);
    
                for (auto* DepthCam : DepthCameras)
                {
                    if (!DepthCam->IsConfigured())
                    {
                        DepthCam->RConfigure(
                            *static_cast<FDepthCameraConfig*>(Component.second.get()),
                            Recorder);
                        // Use both robot tag and camera ID/index for unique identification
                        FString cameraKey = FString::Printf(TEXT("%s^%d"), 
                            *FString(Robot.UETag.c_str()), DepthCam->GetCameraIndex());
                        RGrpcMaps.RCMaps.RDCMap[TCHAR_TO_UTF8(*cameraKey)] = DepthCam;
                    }

                    if (DepthCam->bRecorded)
                    {
                        bHasDepth = true;
                    }
                }
            }
            else if (Component.first == ESensorType::RGBCamera)
            {
                TArray<URGBCameraComponent*> RGBCameras;
                RobotPawn->GetComponents<URGBCameraComponent>(RGBCameras);

                for (auto* RGBCam : RGBCameras)
                {
                    if (!RGBCam->IsConfigured())
                    {
                        RGBCam->RConfigure(
                            *static_cast<FRGBCameraConfig*>(Component.second.get()),
                            Recorder);
                        // Use both robot tag and camera ID/index for unique identification
                        FString cameraKey = FString::Printf(TEXT("%s^%d"), 
                            *FString(Robot.UETag.c_str()), RGBCam->GetCameraIndex());
                        RGrpcMaps.RCMaps.RRGBCMap[TCHAR_TO_UTF8(*cameraKey)] = RGBCam;
                    }

                    if (RGBCam->bRecorded)
                    {
                        bHasRGB = true;
                    }
                }
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT(
                    "AVCCHUD::SetupActors: Unknown component, %d"), Component.first);
            }
        }
        
        Recorder->RegisterPawn(RobotPawn, bHasLidar, bHasDepth, bHasRGB);
    }

    SetupMainCharacter(Config, FoundPawns);
    
    return RGrpcMaps;
}

APawn* AVCCHUD::CreatePawn(const FVCCSimConfig& Config, const FRobot& Robot)
{
    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get World!"));
        return nullptr;
    }
    
    FActorSpawnParameters SpawnParams;
    SpawnParams.SpawnCollisionHandlingOverride =
       ESpawnActorCollisionHandlingMethod::AdjustIfPossibleButAlwaysSpawn;

    APawn* RobotPawn = nullptr;
    UClass* PawnClass = nullptr;
    if (Robot.Type == EPawnType::Drone)
    {
        PawnClass = LoadClass<APawn>(nullptr,
            *FString(FUTF8ToTCHAR(Config.VCCSim.DefaultDronePawn.c_str())));

    }
    else if (Robot.Type == EPawnType::Car)
    {
        PawnClass = LoadClass<APawn>(nullptr,
            *FString(FUTF8ToTCHAR(Config.VCCSim.DefaultCarPawn.c_str())));
    }
    else if (Robot.Type == EPawnType::Flash)
    {
        PawnClass = LoadClass<APawn>(nullptr,
            *FString(FUTF8ToTCHAR(Config.VCCSim.DefaultFlashPawn.c_str())));
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Unknown pawn type!"));
        return nullptr;
    }
    
    if (PawnClass)
    {
        RobotPawn = World->SpawnActor<APawn>(PawnClass, 
            FVector{0, 0, 10}, FRotator::ZeroRotator, SpawnParams);
        if (!RobotPawn)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create Pawn %s"),
                *FString(Robot.UETag.c_str()));
            return nullptr;
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load Drone Blueprint class!"));
        return nullptr;
    }
    
    RobotPawn->Tags.Add(FName(Robot.UETag.c_str()));
    return RobotPawn;
}
