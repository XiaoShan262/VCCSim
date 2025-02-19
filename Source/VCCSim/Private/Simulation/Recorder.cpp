#include "Simulation/Recorder.h"
#include "Sensors/LidarSensor.h"
#include "Kismet/GameplayStatics.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Async/AsyncWork.h"
#include "Serialization/BufferArchive.h"
#include "Serialization/MemoryWriter.h"
#include "Serialization/MemoryReader.h"

ARecorder::ARecorder()
{
    // No need for Tick anymore as we're using event-driven approach
    PrimaryActorTick.bCanEverTick = false;
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
    
    // Create directories
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    PlatformFile.CreateDirectoryTree(*CurrentRecordingPath);
    
    // Initialize directories for each pawn
    for (auto& Pair : RecordComponents)
    {
        auto Pawn = Pair.Key;
        const FRecordComponents& Components = Pair.Value;
        
        FString PawnDir = FPaths::Combine(CurrentRecordingPath, 
            FString::Printf(TEXT("Pawn_%d"), Pawn->GetUniqueID()));
        PlatformFile.CreateDirectoryTree(*PawnDir);
        
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
        
        // Initialize ring buffers for this pawn
        PoseBuffers.Add(Pawn, TRingBuffer<FPoseData>(BufferSize));
        LidarBuffers.Add(Pawn, TRingBuffer<FLidarData>(BufferSize));
        DepthBuffers.Add(Pawn, TRingBuffer<FDepthCameraData>(BufferSize));
        RGBBuffers.Add(Pawn, TRingBuffer<FRGBCameraData>(BufferSize));
    }
    
    // Start the recorder worker
    RecorderWorker = MakeUnique<FRecorderWorker>(CurrentRecordingPath, BufferSize);
    bRecording = true;
}

void ARecorder::StopRecording()
{
    if (!bRecording) return;
    
    bRecording = false;
    
    // Process any remaining data
    SwapAndProcessBuffers();
    
    // Stop and clean up the worker
    if (RecorderWorker)
    {
        RecorderWorker->Stop();
        RecorderWorker.Reset();
    }
    
    // Clear all buffers
    PoseBuffers.Empty();
    LidarBuffers.Empty();
    DepthBuffers.Empty();
    RGBBuffers.Empty();
}

void ARecorder::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    StopRecording();
}

void ARecorder::SubmitPoseData(TSharedPtr<APawn> Pawn, const FPoseData& Data)
{
    if (!bRecording || !Pawn.IsValid()) return;
    
    FScopeLock Lock(&DataLock);
    if (!PoseBuffers.Contains(Pawn))
    {
        PoseBuffers.Add(Pawn, TRingBuffer<FPoseData>(BufferSize));
    }
    
    if (!PoseBuffers[Pawn].Enqueue(Data))
    {
        // Buffer full, trigger processing
        SwapAndProcessBuffers();
        // Try again after processing
        PoseBuffers[Pawn].Enqueue(Data);
    }
}

void ARecorder::SubmitLidarData(TSharedPtr<APawn> Pawn, const FLidarData& Data)
{
    if (!bRecording || !Pawn.IsValid()) return;
    
    FScopeLock Lock(&DataLock);
    if (!LidarBuffers.Contains(Pawn))
    {
        LidarBuffers.Add(Pawn, TRingBuffer<FLidarData>(BufferSize));
    }
    
    if (!LidarBuffers[Pawn].Enqueue(Data))
    {
        SwapAndProcessBuffers();
        LidarBuffers[Pawn].Enqueue(Data);
    }
}

void ARecorder::SubmitDepthData(TSharedPtr<APawn> Pawn, const FDepthCameraData& Data)
{
    if (!bRecording || !Pawn.IsValid()) return;
    
    FScopeLock Lock(&DataLock);
    if (!DepthBuffers.Contains(Pawn))
    {
        DepthBuffers.Add(Pawn, TRingBuffer<FDepthCameraData>(BufferSize));
    }
    
    if (!DepthBuffers[Pawn].Enqueue(Data))
    {
        SwapAndProcessBuffers();
        DepthBuffers[Pawn].Enqueue(Data);
    }
}

void ARecorder::SubmitRGBData(TSharedPtr<APawn> Pawn, const FRGBCameraData& Data)
{
    if (!bRecording || !Pawn.IsValid()) return;
    
    FScopeLock Lock(&DataLock);
    if (!RGBBuffers.Contains(Pawn))
    {
        RGBBuffers.Add(Pawn, TRingBuffer<FRGBCameraData>(BufferSize));
    }
    
    if (!RGBBuffers[Pawn].Enqueue(Data))
    {
        SwapAndProcessBuffers();
        RGBBuffers[Pawn].Enqueue(Data);
    }
}

void ARecorder::SwapAndProcessBuffers()
{
    // Create buffers with accumulated data for each pawn
    for (auto& PawnPair : RecordComponents)
    {
        auto Pawn = PawnPair.Key;
        if (!Pawn.IsValid()) continue;
        
        FPawnBuffer Buffer;
        Buffer.PawnDirectory = FPaths::Combine(CurrentRecordingPath, 
            FString::Printf(TEXT("Pawn_%d"), Pawn->GetUniqueID()));
        Buffer.PoseFilePath = FPaths::Combine(Buffer.PawnDirectory, TEXT("pose.txt"));
        
        // Collect pose data
        if (PoseBuffers.Contains(Pawn))
        {
            FPoseData Data;
            while (PoseBuffers[Pawn].Dequeue(Data))
            {
                Buffer.PoseBuffer.Add(Data);
            }
        }
        
        // Collect Lidar data
        if (LidarBuffers.Contains(Pawn))
        {
            FLidarData Data;
            while (LidarBuffers[Pawn].Dequeue(Data))
            {
                Buffer.LidarBuffer.Add(Data);
            }
        }
        
        // Collect Depth Camera data
        if (DepthBuffers.Contains(Pawn))
        {
            FDepthCameraData Data;
            while (DepthBuffers[Pawn].Dequeue(Data))
            {
                Buffer.DepthCameraBuffer.Add(Data);
            }
        }
        
        // Collect RGB Camera data
        if (RGBBuffers.Contains(Pawn))
        {
            FRGBCameraData Data;
            while (RGBBuffers[Pawn].Dequeue(Data))
            {
                Buffer.RGBCameraBuffer.Add(Data);
            }
        }
        
        // Submit to worker if we have any data
        if (Buffer.PoseBuffer.Num() > 0 || 
            Buffer.LidarBuffer.Num() > 0 ||
            Buffer.DepthCameraBuffer.Num() > 0 ||
            Buffer.RGBCameraBuffer.Num() > 0)
        {
            if (RecorderWorker)
            {
                RecorderWorker->EnqueueData(Buffer);
            }
        }
    }
}

uint32 FRecorderWorker::Run()
{
    while (!bStopRequested)
    {
        FPawnBuffer Data;
        bool HasData = false;
        
        {
            FScopeLock Lock(&QueueLock);
            HasData = DataQueue.Dequeue(Data);
        }
        
        if (HasData)
        {
            // Save pose data
            if (Data.PoseBuffer.Num() > 0)
            {
                FString PoseContent;
                for (const FPoseData& Pose : Data.PoseBuffer)
                {
                    PoseContent += FString::Printf(TEXT("%.6f %.3f %.3f %.3f %.3f %.3f %.3f\n"),
                        Pose.Timestamp,
                        Pose.Location.X, Pose.Location.Y, Pose.Location.Z,
                        Pose.Rotation.Roll, Pose.Rotation.Pitch, Pose.Rotation.Yaw);
                }
                
                FFileHelper::SaveStringToFile(PoseContent, *Data.PoseFilePath, 
                    FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), 
                    FILEWRITE_Append);
            }
            
            // Save Lidar data
            for (const FLidarData& LidarData : Data.LidarBuffer)
            {
                FString Filename = FString::Printf(TEXT("%.6f.dat"), LidarData.Timestamp);
                FString FilePath = FPaths::Combine(Data.PawnDirectory, TEXT("Lidar"), Filename);
                
                // Serialize Lidar data to binary
                TArray<uint8> SerializedData;
                FMemoryWriter MemWriter(SerializedData);
                // MemWriter << LidarData.Data.Num();  // Write array size first
                // for (const FLidarPoint& Point : LidarData.Data)
                // {
                //     // MemWriter << Point;
                // }
                FFileHelper::SaveArrayToFile(SerializedData, *FilePath);
            }
            
            // Save Depth Camera data
            for (const FDepthCameraData& DepthData : Data.DepthCameraBuffer)
            {
                FString Filename = FString::Printf(TEXT("%.6f_%d.dat"), 
                    DepthData.Timestamp, DepthData.SensorIndex);
                FString FilePath = FPaths::Combine(Data.PawnDirectory, TEXT("Depth"), Filename);
                
                TArray<uint8> SerializedData;
                FMemoryWriter MemWriter(SerializedData);
                // MemWriter << DepthData.Data;  // TArray<float> can be serialized directly
                FFileHelper::SaveArrayToFile(SerializedData, *FilePath);
            }
            
            // Save RGB Camera data
            for (const FRGBCameraData& RGBData : Data.RGBCameraBuffer)
            {
                FString Filename = FString::Printf(TEXT("%.6f_%d.dat"), 
                    RGBData.Timestamp, RGBData.SensorIndex);
                FString FilePath = FPaths::Combine(Data.PawnDirectory, TEXT("RGB"), Filename);
                
                TArray<uint8> SerializedData;
                FMemoryWriter MemWriter(SerializedData);
                // MemWriter << RGBData.Data;  // TArray<FColor> can be serialized directly
                FFileHelper::SaveArrayToFile(SerializedData, *FilePath);
            }
        }
        else
        {
            // Sleep briefly if no data to process
            FPlatformProcess::Sleep(0.001f);
        }
    }
    
    return 0;
}