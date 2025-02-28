#include "Sensors/DepthCamera.h"
#include "Simulation/Recorder.h"
#include "RenderingThread.h"
#include "Async/AsyncWork.h"
#include "Windows/WindowsHWrapper.h"
#include "RHI.h"

UDepthCameraComponent::UDepthCameraComponent()
    : FOV(90.0f)
    , MaxRange(2000.0f)
    , MinRange(.0f)
    , Width(512)
    , Height(512)
    , bOrthographic(false)
    , OrthoWidth(512.0f)
    , TimeSinceLastCapture(0.0f)
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UDepthCameraComponent::RConfigure(
    const FDepthCameraConfig& Config, ARecorder* Recorder)
{  
    FOV = Config.FOV;
    MaxRange = Config.MaxRange;
    MinRange = Config.MinRange;
    Width = Config.Width;
    Height = Config.Height;
    bOrthographic = Config.bOrthographic;
    OrthoWidth = Config.OrthoWidth;
    SetCaptureComponent();

    if (Config.RecordInterval > 0)
    {
        ParentActor = GetOwner();
        RecorderPtr = Recorder;
        RecordInterval = Config.RecordInterval;
        RecordState = Recorder->RecordState;
        // UE_LOG(LogTemp, Display, TEXT("RecordState: %d"),RecordState);
        
        Recorder->OnRecordStateChanged.AddDynamic(this,
            &UDepthCameraComponent::SetRecordState);
        SetComponentTickEnabled(true);
        bRecorded = true;
    }
    else
    {
        SetComponentTickEnabled(false);
    }
    bBPConfigured = true;
}

void UDepthCameraComponent::SetCaptureComponent() const
{
    if (CaptureComponent)
    {
        CaptureComponent->ProjectionType = bOrthographic ?
            ECameraProjectionMode::Orthographic : ECameraProjectionMode::Perspective;
        CaptureComponent->FOVAngle = FOV;
        CaptureComponent->OrthoWidth = OrthoWidth;
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        CaptureComponent->bCaptureEveryFrame = false;
        CaptureComponent->bCaptureOnMovement = true;
    }
    else 
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component not initialized!"));
    }
}

void UDepthCameraComponent::BeginPlay()
{
    Super::BeginPlay();
    InitializeRenderTargets();

    // TODO: Is this necessary?
    // Recheck the settings of CaptureComponent
    SetCaptureComponent();
    
    SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SetCollisionResponseToAllChannels(ECR_Ignore);
    SetSimulatePhysics(true);
}

void UDepthCameraComponent::OnComponentCreated()
{
    Super::OnComponentCreated();
    
    // Initialize capture component
    CaptureComponent = NewObject<USceneCaptureComponent2D>(this);
    CaptureComponent->AttachToComponent(this,
        FAttachmentTransformRules::SnapToTargetIncludingScale);
    
    SetCaptureComponent();
}

void UDepthCameraComponent::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bRecorded && RecordState)
    {
        TimeSinceLastCapture += DeltaTime;
        if (TimeSinceLastCapture >= RecordInterval)
        {
            TimeSinceLastCapture = 0.0f;
            CaptureDepthScene();
            if (RecorderPtr)
            {
                FDepthCameraData DepthCameraData;
                DepthCameraData.Timestamp = FPlatformTime::Seconds();
                DepthCameraData.SensorIndex = CameraIndex;
                DepthCameraData.Width = Width;
                DepthCameraData.Height = Height;
                DepthCameraData.Data = GetDepthImage();
                RecorderPtr->SubmitDepthData(ParentActor, MoveTemp(DepthCameraData));
            }
        }
    }
}

void UDepthCameraComponent::InitializeRenderTargets()
{
    DepthRenderTarget = NewObject<UTextureRenderTarget2D>(this);
    DepthRenderTarget->InitCustomFormat(Width, Height,
        PF_FloatRGBA, true);
    
    DepthRenderTarget->UpdateResource();
    CaptureComponent->TextureTarget = DepthRenderTarget;
    if (CaptureComponent==nullptr)
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component not initialized!"));
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("Capture component initialized success!"));
    }
}

bool UDepthCameraComponent::CheckComponentAndRenderTarget() const
{
    if (!CaptureComponent || !DepthRenderTarget)
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component or "
                   "render target not initialized!"));
        return true;
    }
    return false;
}

TArray<FDCPoint> UDepthCameraComponent::GetPointCloudDataGameThread()
{
    check(IsInGameThread());
    
    if (CheckComponentAndRenderTarget()) return {};

    UE_LOG(LogTemp, Warning, TEXT("Depth camera data ready!"));
    
    CaptureComponent->CaptureScene();
    ProcessDepthTexture();
    return GeneratePointCloud();
}

TArray<float> UDepthCameraComponent::GetDepthImageDataGameThread()
{
    check(IsInGameThread());
    
    if (CheckComponentAndRenderTarget()) return {};

    CaptureComponent->CaptureScene();
    ProcessDepthTexture();
    
    return GetDepthImage();
}

void UDepthCameraComponent::CaptureDepthScene()
{
    if (CheckComponentAndRenderTarget())
    {
        UE_LOG(LogTemp, Error, TEXT("UDepthCameraComponent: "
                                    "Capture component or render target not initialized!"));
        return;
    }
    
    CaptureComponent->CaptureScene();
    
    ProcessDepthTexture();
    PointCloudData = GeneratePointCloud();
}

// void UDepthCameraComponent::ProcessDepthTexture()
// {
//     FTextureRenderTargetResource* RenderTargetResource =
//         DepthRenderTarget->GameThread_GetRenderTargetResource();
//     
//     if (!RenderTargetResource)
//     {
//         UE_LOG(LogTemp, Error, TEXT("Failed to get render target resource!"));
//         return;
//     }
//     DepthData.Empty(Width * Height);
//     
//     struct FReadSurfaceContext
//     {
//         TArray<FFloat16Color>* OutData;
//         FTextureRenderTargetResource* RenderTarget;
//         FIntRect Rect;
//         FReadSurfaceDataFlags Flags;
//     };
//
//     FReadSurfaceContext Context = {
//         &DepthData,
//         RenderTargetResource,
//         FIntRect(0, 0, Width, Height),
//         FReadSurfaceDataFlags(RCM_MinMax, CubeFace_MAX)
//     };
//     // UE_LOG(LogTemp,Log,"")
//     ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)(
//         [Context](FRHICommandListImmediate& RHICmdList)
//         {
//             RHICmdList.ReadSurfaceFloatData(
//                 Context.RenderTarget->GetRenderTargetTexture(),
//                 Context.Rect,
//                 *Context.OutData,
//                 ECubeFace::CubeFace_PosX,
//                 0,
//                 0
//             );
//         });
//     
//     // Wait for the rendering thread to finish
//     FlushRenderingCommands();
//     
//     // Check success 
//     // UE_LOG(LogTemp, Warning, TEXT("ProcessDepthTexture Success!"));
// }

void UDepthCameraComponent::ProcessDepthTexture()
{
    // Get the render target resource
    FTextureRenderTargetResource* RenderTargetResource = 
        DepthRenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get render target resource!"));
        return;
    }

    // Get the current write buffer index
    const int32 WriteBufferIndex = CurrentWriteBufferIndex.load();
    TArray<FFloat16Color>& WriteBuffer = DepthDataBuffers[WriteBufferIndex];
    WriteBuffer.Empty(Width * Height);

    // Define context for reading surface data
    struct FReadSurfaceContext
    {
        TArray<FFloat16Color>* OutData;
        FTextureRenderTargetResource* RenderTarget;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    FReadSurfaceContext Context = {
        &WriteBuffer,
        RenderTargetResource,
        FIntRect(0, 0, Width, Height),
        FReadSurfaceDataFlags(RCM_MinMax, CubeFace_MAX)
    };
    
    // Submit the render thread command
    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([Context, this, WriteBufferIndex](FRHICommandListImmediate& RHICmdList)
    {
        // The render thread performs the data read
        RHICmdList.ReadSurfaceFloatData(
            Context.RenderTarget->GetRenderTargetTexture(),
            Context.Rect,
            *Context.OutData,
            ECubeFace::CubeFace_PosX,
            0,
            0
        );
        // Switch the buffer index (atomic operation)
        const int32 NewBufferIndex = (WriteBufferIndex + 1) % 2;
        CurrentWriteBufferIndex.store(NewBufferIndex);
        // Mark data as ready
        bDataReady.store(true);
        // Dispatch the data processing task to the main thread
        FFunctionGraphTask::CreateAndDispatchWhenReady([this]()
        {
            if (bDataReady.load())
            {
                this->OnDepthDataProcessed();
                bDataReady.store(false);
            }
        },
        TStatId(),
        nullptr,
        ENamedThreads::GameThread);
    });
}

void UDepthCameraComponent::OnDepthDataProcessed()
{
    // Get the index of the buffer to be read (atomic operation)
    const int32 ReadBufferIndex = (CurrentWriteBufferIndex.load() + 1) % 2;
    const TArray<FFloat16Color>& ReadBuffer = DepthDataBuffers[ReadBufferIndex];

    // Lock to protect DepthData writes
    FScopeLock Lock(&DataLock);

    // Copy double-buffered data to the main thread's DepthData
    DepthData = ReadBuffer; 
}


TArray<FDCPoint> UDepthCameraComponent::GeneratePointCloud()
{
    if (DepthData.Num() == 0) 
    {
        UE_LOG(LogTemp, Error, TEXT("GeneratePointCloud: No depth data available!"));
        return {};
    }

    TArray<FDCPoint> tPointCloudData;

    // Get camera transform
    const FTransform CameraTransform = GetComponentTransform();
    const float HalfFOV = FMath::DegreesToRadians(FOV * 0.5f);
    const float AspectRatio = static_cast<float>(Width) / Height;

    for (int32 Y = 0; Y < Height; ++Y)
    {
        for (int32 X = 0; X < Width; ++X)
        {
            const int32 Index = Y * Width + X;
            const float Depth = DepthData[Index].R.GetFloat();

            // Skip invalid depth values
            if (Depth < MinRange || Depth > MaxRange) continue;

            FDCPoint Point;
            FVector WorldPos;

            if (bOrthographic)
            {
                // Convert pixel coordinates to world space
                const float WorldX =
                    (static_cast<float>(X) / Width - 0.5f) * OrthoWidth;
                const float WorldY =
                    (static_cast<float>(Y) / Height - 0.5f) * (OrthoWidth / AspectRatio);

                // Create point in camera space (Forward, Right, Up)
                FVector CameraSpacePos(Depth, WorldX, -WorldY);

                // Transform to world space
                WorldPos = CameraTransform.TransformPosition(CameraSpacePos);
            }
            else
            {
                // New perspective calculation
                // Range -1 to 1
                float ScreenX = static_cast<float>(X) / Width * 2.0f - 1.0f;
                // Range 1 to -1
                float ScreenY = 1.0f - static_cast<float>(Y) / Height * 2.0f;
                
                // Scale by FOV
                ScreenX *= AspectRatio * FMath::Tan(HalfFOV);
                ScreenY *= FMath::Tan(HalfFOV);

                // Create camera space position
                FVector CameraSpacePos(
                    Depth,                  // Forward
                    ScreenX * Depth,        // Right
                    ScreenY * Depth         // Up
                );
                
                // Transform to world space
                WorldPos = CameraTransform.TransformPosition(CameraSpacePos);
            }

            Point.Location = WorldPos;
            tPointCloudData.Add(Point);
        }
    }
    // Clear the depth data
    // DepthData.Empty(Width * Height);

    return tPointCloudData;
}

TArray<float> UDepthCameraComponent::GetDepthImage()
{
    TArray<float> DepthImage;
    DepthImage.Empty(Width * Height);

    // Wait if depth data hasn't been processed yet
    if (DepthData.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("GetDepthImage: No depth data available!"));
        return DepthImage;
    }
    
    // Check success: Depth Data always be empty
    // UE_LOG(LogTemp, Log, TEXT("GetDepthImage: Depth data is available!!"));
    
    // Convert FFloat16Color to float depth values
    for (const FFloat16Color& Color : DepthData)
    {
        // Usually depth is stored in the R channel for single-channel depth
        DepthImage.Add(Color.R.GetFloat());
    }

    return DepthImage;
}

void UDepthCameraComponent::VisualizePointCloud()
{
    // Draw the point cloud. Debug only
    UWorld* World = GetWorld();
    if (!World) return;
    for (const FDCPoint& Point : PointCloudData)
    {
        DrawDebugPoint(World, Point.Location, 5.0f, FColor::Red,
            false, -1.0f);
    }
}

void UDepthCameraComponent::AsyncGetPointCloudData(
    TFunction<void(TArray<FDCPoint>)> Callback)
{
    AsyncTask(ENamedThreads::GameThread, [this, Callback]() {
        TArray<FDCPoint> PointCloud = GetPointCloudDataGameThread();
        Callback(PointCloud);
    });
}

void UDepthCameraComponent::AsyncGetDepthImageData(
    TFunction<void(TArray<float>)> Callback)
{
    AsyncTask(ENamedThreads::GameThread, [this, Callback]() {
        TArray<float> DepthImage = GetDepthImageDataGameThread();
        Callback(DepthImage);
    });
}
