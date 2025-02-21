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
#include "GameFramework/Actor.h"
#include "SensorBase.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Materials/MaterialInterface.h"
#include "RHIResources.h"
#include "CameraSensor.generated.h"

class ARecorder;

class FRGBCameraConfig : public FSensorConfig
{
public:
    float FOV = 90.0f;
    int32 Width = 512;
    int32 Height = 512;
    bool bOrthographic = false;
    float OrthoWidth = 512.0f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
    FOnRGBImageCaptured, const TArray<FColor>&, ImageData);

UCLASS(ClassGroup = (VCCSIM), meta = (BlueprintSpawnableComponent))
class VCCSIM_API URGBCameraComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    URGBCameraComponent();
    void RConfigure(const FRGBCameraConfig& Config, ARecorder* Recorder);
    bool IsConfigured() const { return bBPConfigured; }
    UFUNCTION()
    void SetRecordState(bool RState) { RecordState = RState; }
    
    int32 GetCameraIndex() const { return CameraIndex; }
    
    void SetCaptureComponent() const;
    void InitializeRenderTargets();
    
    void ProcessRGBTextureAsyncRaw();
    void ProcessRGBTextureAsync(TFunction<void(const TArray<FColor>&)> OnComplete);

    UFUNCTION(BlueprintCallable, Category = "RGBCamera")
    void CaptureRGBScene();
    
    // For GRPC call
    void AsyncGetRGBImageData(TFunction<void(const TArray<FColor>&)> Callback);

protected:
    virtual void BeginPlay() override;
    virtual void OnComponentCreated() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

public:
    // Configuration Properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    float FOV;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    int32 Width;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    int32 Height;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    bool bOrthographic;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config", 
        meta = (EditCondition = "bOrthographic"))
    float OrthoWidth;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    bool bBPConfigured = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Config")
    int32 CameraIndex = 0;

    UPROPERTY(BlueprintAssignable, Category = "Camera")
    FOnRGBImageCaptured OnRGBImageCaptured;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "RGBCamera|Debug")
    bool bRecorded = false;
    
    UPROPERTY()
    UTextureRenderTarget2D* RGBRenderTarget = nullptr;
    
private:
    bool CheckComponentAndRenderTarget() const;
    void CaptureRGBImageAsync();

    struct FReadSurfaceContext
    {
        TArray<FColor>* OutData;
        FTextureRenderTargetResource* RenderTarget;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };
    
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent = nullptr;
    TArray<FColor> RGBData;
    FCriticalSection DataLock;

    UPROPERTY()
    AActor* ParentActor = nullptr;
    UPROPERTY()
    ARecorder* RecorderPtr = nullptr;
    bool RecordState = false;
    float RecordInterval = -1.f;
    float TimeSinceLastCapture;
};