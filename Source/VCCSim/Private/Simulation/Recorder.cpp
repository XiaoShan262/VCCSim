#include "Simulation/Recorder.h"
#include "Kismet/GameplayStatics.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/CameraSensor.h"

ARecorder::ARecorder()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ARecorder::StartRecording()
{
    if (bRecording) return;
    
    // Create base directory with timestamp
    FDateTime Now = FDateTime::Now();
    CurrentRecordingPath = FPaths::Combine(
        LogBasePath,
        Now.ToString(TEXT("%Y%m%d_%H%M%S"))
    );
    
    InitializeRecordingDirectories();
    bRecording = true;
}

void ARecorder::StopRecording()
{
    if (!bRecording) return;
    
    bRecording = false;
    FlushAllBuffers();
}

void ARecorder::InitializeRecordingDirectories()
{
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    
    // Create base directory
    PlatformFile.CreateDirectoryTree(*CurrentRecordingPath);
    
    // Initialize directories for each pawn
    for (auto& Pair : RecordComponents)
    {
        APawn* Pawn = Pair.Key;
        const FRecordComponents& Components = Pair.Value;
        
        FString PawnDir = FPaths::Combine(CurrentRecordingPath, 
            FString::Printf(TEXT("Pawn_%d"), Pawn->GetUniqueID()));
        PlatformFile.CreateDirectoryTree(*PawnDir);
        
        FPawnBuffer& Buffer = PawnBuffers.Add(Pawn);
        Buffer.PawnDirectory = PawnDir;
        Buffer.PoseFilePath = FPaths::Combine(PawnDir, TEXT("pose.txt"));
        
        // Create sensor subdirectories if needed
        if (Components.LidarComponents.Num() > 0)
        {
            PlatformFile.CreateDirectoryTree(*FPaths::Combine(PawnDir, TEXT("Lidar")));
        }
        if (Components.DepthCameraComponents.Num() > 0)
        {
            PlatformFile.CreateDirectoryTree(*FPaths::Combine(PawnDir, TEXT("Depth")));
        }
        if (Components.RGBCameraComponents.Num() > 0)
        {
            PlatformFile.CreateDirectoryTree(*FPaths::Combine(PawnDir, TEXT("RGB")));
        }
    }
}

void ARecorder::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    
    if (!bRecording) return;
    
    TimeSinceLastRecord += DeltaTime;
    if (TimeSinceLastRecord >= RecordRate)
    {
        RecordFrame();
        TimeSinceLastRecord = 0.0f;
    }
}

void ARecorder::RecordFrame()
{
    double CurrentTime = FPlatformTime::Seconds();
    
    for (auto& Pair : RecordComponents)
    {
        APawn* Pawn = Pair.Key;
        const FRecordComponents& Components = Pair.Value;
        FPawnBuffer& Buffer = PawnBuffers[Pawn];
        
        // Record pose
        FPoseData PoseData;
        PoseData.Timestamp = CurrentTime;
        PoseData.Location = Pawn->GetActorLocation();
        PoseData.Rotation = Pawn->GetActorRotation();
        
        FScopeLock Lock(&DataLock);
        Buffer.PoseBuffer.Add(PoseData);
        
        // Record from each sensor
        for (int32 i = 0; i < Components.LidarComponents.Num(); ++i)
        {
            FLidarData LidarData;
            LidarData.Timestamp = CurrentTime;
            LidarData.Data = Components.LidarComponents[i]->GetPointCloudData();
            Buffer.LidarBuffer.Add(LidarData);
        }
        for (int32 i = 0; i < Components.DepthCameraComponents.Num(); ++i)
        {
            FDepthCameraData DepthData;
            DepthData.Timestamp = CurrentTime;
            DepthData.Data = Components.DepthCameraComponents[i]->GetDepthImageDataGameThread();
            DepthData.SensorIndex = i;
            Buffer.DepthCameraBuffer.Add(DepthData);
        }
        for (int32 i = 0; i < Components.RGBCameraComponents.Num(); ++i)
        {
            FRGBCameraData RGBData;
            RGBData.Timestamp = CurrentTime;
            // RGBData.Data = Components.RGBCameraComponents[i]->GetRGBImageDataGameThread();
            RGBData.SensorIndex = i;
            Buffer.RGBCameraBuffer.Add(RGBData);
        }
        
        // Check if buffer needs to be flushed
        if (Buffer.PoseBuffer.Num() >= BufferSize)
        {
            SavePawnData(Pawn, Buffer);
            Buffer.PoseBuffer.Empty();
            Buffer.LidarBuffer.Empty();
            Buffer.DepthCameraBuffer.Empty();
            Buffer.RGBCameraBuffer.Empty();
        }
    }
}

void ARecorder::SavePawnData(APawn* Pawn, const FPawnBuffer& Buffer)
{
    // Save pose data
    FString PoseContent;
    for (const FPoseData& Pose : Buffer.PoseBuffer)
    {
        PoseContent += FString::Printf(TEXT("%.6f %.3f %.3f %.3f %.3f %.3f %.3f\n"),
            Pose.Timestamp,
            Pose.Location.X, Pose.Location.Y, Pose.Location.Z,
            Pose.Rotation.Roll, Pose.Rotation.Pitch, Pose.Rotation.Yaw);
    }
    
    // Append to pose file asynchronously
    (new FAutoDeleteAsyncTask<FAsyncSaveTask>(
        Buffer.PoseFilePath,
        TArray<uint8>((uint8*)TCHAR_TO_UTF8(*PoseContent), PoseContent.Len())
    ))->StartBackgroundTask();
    
    // Save sensor data asynchronously
    // for (const FSensorData& SensorData : Buffer.SensorBuffer)
    // {
    //     FString Filename = FString::Printf(TEXT("%.6f"), SensorData.Timestamp);
    //     if (SensorData.SensorIndex >= 0)
    //     {
    //         Filename += FString::Printf(TEXT("_%d"), SensorData.SensorIndex);
    //     }
    //     
    //     FString FilePath = FPaths::Combine(Buffer.PawnDirectory, 
    //         SensorData.SensorType, 
    //         Filename);
    //         
    //     (new FAutoDeleteAsyncTask<FAsyncSaveTask>(
    //         FilePath,
    //         SensorData.Data
    //     ))->StartBackgroundTask();
    // }
}

void ARecorder::FlushAllBuffers()
{
    for (auto& Pair : PawnBuffers)
    {
        SavePawnData(Pair.Key, Pair.Value);
    }
    PawnBuffers.Empty();
}

void ARecorder::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    StopRecording();
}