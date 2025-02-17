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


struct FDCPoint
{
    FVector Location;
    FDCPoint() : Location(FVector::ZeroVector){}
};

class DepthCameraConfig: public SensorConfig
{
public:
    float FOV = 90.0f;
    float MaxRange = 2000.0f;
    float MinRange = .0f;
    int32 Width = 512;
    int32 Height = 512;
    bool bOrthographic = false;
    float OrthoWidth = 512.0f;
    float CaptureRate = 30.f;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VCCSIM_API UDepthCameraComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    UDepthCameraComponent();
    void DCConfigure(const DepthCameraConfig& Config);
    bool IsConfigured() const { return bBPConfigured; }
    int32 GetCameraIndex() const { return CameraIndex; }
    void SetCaptureComponent() const;

    UFUNCTION(BlueprintCallable, Category = "DepthCamera")
    void CaptureDepthScene();
    UFUNCTION(BlueprintCallable, Category = "DepthCamera")
    void VisualizePointCloud();

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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Performance")
    bool bAutoCapture;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DepthCamera|Performance")
    float CaptureRate;

    UPROPERTY()
    UTextureRenderTarget2D* DepthRenderTarget = nullptr;
    
private:
    bool CheckComponentAndRenderTarget() const;
    TArray<FDCPoint> GetPointCloudDataGameThread();
    TArray<float> GetDepthImageDataGameThread();
    
    UPROPERTY()
    USceneCaptureComponent2D* CaptureComponent = nullptr;
    TArray<FDCPoint> PointCloudData;
    TArray<FFloat16Color> DepthData;
    float TimeSinceLastCapture;
    FCriticalSection DataLock;
};