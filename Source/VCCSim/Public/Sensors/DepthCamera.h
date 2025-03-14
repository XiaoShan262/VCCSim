﻿/*
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
#include "GameFramework/Actor.h"
#include "SensorBase.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Materials/MaterialInterface.h"
#include "RHIResources.h"
#include "DepthCamera.generated.h"

class ARecorder;

struct FDCPoint
{
    FVector Location;
    FDCPoint() : Location(FVector::ZeroVector){}
};

class FDepthCameraConfig: public FSensorConfig
{
public:
    float FOV = 90.0f;
    float MaxRange = 2000.0f;
    float MinRange = .0f;
    int32 Width = 512;
    int32 Height = 512;
    bool bOrthographic = false;
    float OrthoWidth = 512.0f;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VCCSIM_API UDepthCameraComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    UDepthCameraComponent();
    void RConfigure(const FDepthCameraConfig& Config, ARecorder* Recorder);
    bool IsConfigured() const { return bBPConfigured; }
    UFUNCTION()
    void SetRecordState(bool RState){ RecordState = RState; }
    
    int32 GetCameraIndex() const { return CameraIndex; }
    
    void SetCaptureComponent() const;

    UFUNCTION(BlueprintCallable, Category = "DepthCamera")
    void CaptureDepthScene();
    UFUNCTION(BlueprintCallable, Category = "DepthCamera")
    void VisualizePointCloud();

    TArray<FDCPoint> GetPointCloudDataGameThread();
    TArray<float> GetDepthImageDataGameThread();
    // For grpc server
    void AsyncGetPointCloudData(TFunction<void(TArray<FDCPoint>)> Callback);
    void AsyncGetDepthImageData(TFunction<void(TArray<float>)> Callback);
    
protected:
    virtual void BeginPlay() override;
    virtual void OnComponentCreated() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction) override;
    
    void InitializeRenderTargets();
    void ProcessDepthTexture();
    void OnDepthDataProcessed();
    TArray<FDCPoint> GeneratePointCloud();
    TArray<float> GetDepthImage();
    
public:
    // Configuration Properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    float FOV;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    float MaxRange;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    float MinRange;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    int32 Width;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    int32 Height;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    bool bOrthographic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config", 
        meta = (EditCondition = "bOrthographic"))
    float OrthoWidth;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    bool bBPConfigured = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Config")
    int32 CameraIndex = 0;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Lidar|Debug")
    bool bRecorded = false;

    UPROPERTY()
    UTextureRenderTarget2D* DepthRenderTarget = nullptr;
    
private:
    bool CheckComponentAndRenderTarget() const;
    
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent = nullptr;
    TArray<FDCPoint> PointCloudData;
    TArray<FFloat16Color> DepthData;
    FCriticalSection DataLock;
    
    UPROPERTY()
    AActor* ParentActor = nullptr;
    UPROPERTY()
    ARecorder* RecorderPtr = nullptr;
    float RecordInterval = -1.f;
    bool RecordState = false;
    float TimeSinceLastCapture;

    // Double-buffered data: one for the rendering thread to write to, the other for the main thread to read from
    TArray<FFloat16Color> DepthDataBuffers[2];
    std::atomic<int32> CurrentWriteBufferIndex = 0;
    std::atomic<bool> bDataReady = false;
};