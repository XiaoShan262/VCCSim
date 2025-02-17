#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SensorBase.h"
#include "Utils/InsMeshHolder.h"
#include "LidarSensor.generated.h"


struct FLidarPoint
{
    FVector Location = FVector::ZeroVector;
    bool bHit = false;
};

class LiDARConfig : public SensorConfig
{
public:
    int32 NumRays = 32;
    int32 NumPoints = 3200;
    double ScannerRangeInner = 300;
    double ScannerRangeOuter = 8000;
    double ScannerAngleUp = 30;
    double ScannerAngleDown = 30;
    bool bVisualizePoints = true;
};

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class VCCSIM_API ULidarComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    
    ULidarComponent();
    void LCConfigure(const LiDARConfig& Config);

    UFUNCTION(BlueprintCallable, Category = "Lidar")
    void FirstCall();
    void InitSensor();

    UFUNCTION(BlueprintCallable, Category = "Lidar")
    void VisualizePointCloud();

    // For grpc server
    TArray<FLidarPoint> GetPointCloudData();
    TPair<TArray<FLidarPoint>, FVCCSimOdom> GetPointCloudDataAndOdom();

protected:
    
    virtual void BeginPlay() override;
    virtual void OnComponentCreated() override;
    TArray<FLidarPoint> PerformLineTraces(FVCCSimOdom* Odom = nullptr);

public:

    // Properties can be set in the editor and config file.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    int32 NumRays = 32;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    int32 NumPoints = 3200;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    double ScannerRangeInner = 300;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    double ScannerRangeOuter = 8000;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    double ScannerAngleUp = 30;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    double ScannerAngleDown = 30;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    bool bVisualizePoints = true;

    // Constructor properties
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Lidar|Debug")
    int ActualNumPoints = -1;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Config")
    bool bReturnUnHitPoints = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LiDAR|Performance")
    float UpdateThresholdDistance = 1.f;  // In centimeters
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LiDAR|Performance")
    float UpdateThresholdAngle = 0.5; // In degrees
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LiDAR|Performance")
    int32 ChunkSize = 256;

    UPROPERTY()
    UInsMeshHolder* MeshHolder;

private:
    static constexpr int32 CACHE_LINE_SIZE = 64;

    TArray<FVector> LocalStartPoints;
    TArray<FVector> LocalEndPoints;
    
    FVector LastLocation; 
    FRotator LastRotation;
    TArray<FVector> CachedStartPoints;
    TArray<FVector> CachedEndPoints;
    TArray<FLidarPoint> PointPool;
    
    FCollisionQueryParams QueryParams;
    
    int32 NumChunks = 0;
    TArray<int32> ChunkStartIndices;
    TArray<int32> ChunkEndIndices;
    
    void ProcessChunk(int32 ChunkIndex);
    void UpdateCachedPoints(const FVector& NewLocation,
        const FRotator& NewRotation);
    bool ShouldUpdateCache(const FVector& NewLocation,
        const FRotator& NewRotation) const;
    TArray<FTransform> GetHitTransforms() const;
};