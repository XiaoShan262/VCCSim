#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Containers/Queue.h"
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

template<typename T>
class TRingBuffer
{
public:
    explicit TRingBuffer(int32 Size) : MaxSize(Size), Head(0), Tail(0)
    {
        Buffer.SetNum(Size);
    }

    bool Enqueue(const T& Item)
    {
        const int32 NewTail = (Tail + 1) % MaxSize;
        if (NewTail == Head) return false; // Buffer full
        
        Buffer[Tail] = Item;
        Tail = NewTail;
        return true;
    }

    bool Dequeue(T& OutItem)
    {
        if (Head == Tail) return false; // Buffer empty
        
        OutItem = Buffer[Head];
        Head = (Head + 1) % MaxSize;
        return true;
    }

    void Reset() { Head = Tail = 0; }
    bool IsEmpty() const { return Head == Tail; }

private:
    TArray<T> Buffer;
    int32 MaxSize;
    int32 Head;
    int32 Tail;
};

class FRecorderWorker : public FRunnable
{
public:
    FRecorderWorker(const FString& InBasePath, int32 InBufferSize)
        : BasePath(InBasePath)
        , BufferSize(InBufferSize)
        , bStopRequested(false)
    {
        Thread = FRunnableThread::Create(
            this, TEXT("RecorderWorker"));
    }

    virtual ~FRecorderWorker()
    {
        Stop();
        if (Thread)
        {
            Thread->WaitForCompletion();
            delete Thread;
        }
    }

    // FRunnable interface
    virtual bool Init() override { return true; }
    virtual uint32 Run() override;
    virtual void Stop() override { bStopRequested = true; }

    void EnqueueData(const FPawnBuffer& Data)
    {
        FScopeLock Lock(&QueueLock);
        DataQueue.Enqueue(Data);
    }

private:
    FString BasePath;
    int32 BufferSize;
    FRunnableThread* Thread;
    FCriticalSection QueueLock;
    TQueue<FPawnBuffer> DataQueue;
    TAtomic<bool> bStopRequested;
};

UCLASS()
class VCCSIM_API ARecorder : public AActor
{
    GENERATED_BODY()
public:
    ARecorder();
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    
    void StartRecording();
    void StopRecording();

    // Called by sensor components to submit data
    void SubmitPoseData(TSharedPtr<APawn> Pawn, const FPoseData& Data);
    void SubmitLidarData(TSharedPtr<APawn> Pawn, const FLidarData& Data);
    void SubmitDepthData(TSharedPtr<APawn> Pawn, const FDepthCameraData& Data);
    void SubmitRGBData(TSharedPtr<APawn> Pawn, const FRGBCameraData& Data);

    TMap<TSharedPtr<APawn>, FRecordComponents> RecordComponents;

private:
    UPROPERTY(EditAnywhere, Category = "Recording")
    int32 BufferSize = 100;
    
    UPROPERTY(EditAnywhere, Category = "Recording")
    FString LogBasePath = FPaths::ProjectSavedDir() / TEXT("Recordings");

    bool bRecording = false;
    FString CurrentRecordingPath;
    
    // Double buffering for each pawn
    TMap<TSharedPtr<APawn>, TRingBuffer<FPoseData>> PoseBuffers;
    TMap<TSharedPtr<APawn>, TRingBuffer<FLidarData>> LidarBuffers;
    TMap<TSharedPtr<APawn>, TRingBuffer<FDepthCameraData>> DepthBuffers;
    TMap<TSharedPtr<APawn>, TRingBuffer<FRGBCameraData>> RGBBuffers;
    
    FCriticalSection DataLock;
    TUniquePtr<FRecorderWorker> RecorderWorker;

    void SwapAndProcessBuffers();
};
