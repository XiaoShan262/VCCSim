#pragma once

#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Async/AsyncWork.h"
#include "Sensors/SensorBase.h"
#include "Recorder.generated.h"


class ULidarComponent;
class UDepthCameraComponent;
class URGBCameraComponent;
struct FLidarPoint;

struct FRecordComponents
{
    TArray<ULidarComponent*> LidarComponents;
    TArray<UDepthCameraComponent*> DepthCameraComponents;
    TArray<URGBCameraComponent*> RGBCameraComponents;
};

// Structure to hold pose data
struct FPoseData
{
    double Timestamp;
    FVector Location;
    FRotator Rotation;
};

// Structure to hold sensor data for async saving
struct FLidarData
{
    double Timestamp;
    TArray<FLidarPoint> Data;
};

struct FDepthCameraData
{
    double Timestamp;
    TArray<float> Data;
    int32 SensorIndex;
};

struct FRGBCameraData
{
    double Timestamp;
    TArray<FColor> Data;
    int32 SensorIndex;
};

// Structure to buffer data for each pawn
struct FPawnBuffer
{
    TArray<FPoseData> PoseBuffer;
    TArray<FLidarData> LidarBuffer;
    TArray<FDepthCameraData> DepthCameraBuffer;
    TArray<FRGBCameraData> RGBCameraBuffer;
    FString PawnDirectory;
    FString PoseFilePath;
};

UCLASS()
class VCCSIM_API ARecorder : public AActor
{
    GENERATED_BODY()
public:
    ARecorder();
    virtual void Tick(float DeltaTime) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    
    void StartRecording();
    void StopRecording();

    TMap<APawn*, FRecordComponents> RecordComponents;

private:
    // Configuration
    UPROPERTY(EditAnywhere, Category = "Recording")
    float RecordRate = 0.2f;
    
    UPROPERTY(EditAnywhere, Category = "Recording")
    int32 BufferSize = 100; // Number of frames to buffer before saving
    
    UPROPERTY(EditAnywhere, Category = "Recording")
    FString LogBasePath = FPaths::ProjectSavedDir() / TEXT("Recordings");

    // State variables
    float TimeSinceLastRecord = 0.0f;
    bool bRecording = false;
    FString CurrentRecordingPath;
    TMap<APawn*, FPawnBuffer> PawnBuffers;
    FCriticalSection DataLock;

    // Helper functions
    void InitializeRecordingDirectories();
    void RecordFrame();
    void SavePawnData(APawn* Pawn, const FPawnBuffer& Buffer);
    void FlushAllBuffers();
    
    // Async save task
    class FAsyncSaveTask : public FNonAbandonableTask
    {
    public:
        FAsyncSaveTask(const FString& FilePath, const TArray<uint8>& InData)
            : Path(FilePath), Data(InData) {}

        void DoWork()
        {
            FFileHelper::SaveArrayToFile(Data, *Path);
        }
        FORCEINLINE TStatId GetStatId() const
        {
            RETURN_QUICK_DECLARE_CYCLE_STAT(FAsyncSaveTask, STATGROUP_ThreadPoolAsyncTasks);
        }
    private:
        FString Path;
        TArray<uint8> Data;
    };
};