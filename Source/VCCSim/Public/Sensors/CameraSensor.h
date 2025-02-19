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


class FRGBCameraConfig : public SensorConfig
{
public:
    float FOV = 90.0f;
    int32 Width = 512;
    int32 Height = 512;
    bool bOrthographic = false;
    float OrthoWidth = 512.0f;
    float CaptureRate = 30.f;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(
    FOnRGBImageCaptured, const TArray<FColor>&, ImageData);

UCLASS(ClassGroup = (VCCSIM))
class VCCSIM_API URGBCameraComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    URGBCameraComponent();
    void RGBConfigure(const FRGBCameraConfig& Config);
    bool IsConfigured() const { return bBPConfigured; }
    
    int32 GetCameraIndex() const { return CameraIndex; }
    
    void SetCaptureComponent() const;
    void InitializeRenderTargets();
    void ProcessRGBTextureAsyncRaw();
    void ProcessRGBTextureAsync(TFunction<void(const TArray<FColor>&)> OnComplete);

    UFUNCTION(BlueprintCallable, Category = "RGBCamera")
    void CaptureRGBScene();

    // For Recorder
    // TArray<FColor> 
    
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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Performance")
    bool bAutoCapture;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RGBCamera|Performance")
    float CaptureRate;

    UPROPERTY(BlueprintAssignable, Category = "Camera")
    FOnRGBImageCaptured OnRGBImageCaptured;
    
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
    float TimeSinceLastCapture;
    FCriticalSection DataLock;
};