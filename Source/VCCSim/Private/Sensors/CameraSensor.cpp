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

#include "Sensors/CameraSensor.h"
#include "Simulation/Recorder.h"
#include "RenderingThread.h"
#include "Async/AsyncWork.h"
#include "Windows/WindowsHWrapper.h"

URGBCameraComponent::URGBCameraComponent()
    : FOV(90.0f)
    , Width(512)
    , Height(512)
    , bOrthographic(false)
    , OrthoWidth(512.0f)
    , TimeSinceLastCapture(0.0f)
{
    PrimaryComponentTick.bCanEverTick = true;
}

void URGBCameraComponent::BeginPlay()
{
    Super::BeginPlay();
    
    InitializeRenderTargets();
    SetCaptureComponent();
    
    SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    SetCollisionResponseToAllChannels(ECR_Ignore);
    SetSimulatePhysics(true);
}

void URGBCameraComponent::OnComponentCreated()
{
    Super::OnComponentCreated();
    
    CaptureComponent = NewObject<USceneCaptureComponent2D>(this);
    CaptureComponent->AttachToComponent(this,
        FAttachmentTransformRules::SnapToTargetIncludingScale);
    
    SetCaptureComponent();
}

void URGBCameraComponent::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    
    if (bRecorded)
    {
        TimeSinceLastCapture += DeltaTime;
        if (TimeSinceLastCapture >= RecordInterval)
        {
            TimeSinceLastCapture = 0.0f;
            CaptureRGBScene();
            ProcessRGBTextureAsyncRaw();
            if (RecorderPtr)
            {
                FRGBCameraData DepthData;
                DepthData.Timestamp = FPlatformTime::Seconds();
                DepthData.SensorIndex = CameraIndex;
                DepthData.Width = Width;
                DepthData.Height = Height;
                DepthData.Data = RGBData;
                RecorderPtr->SubmitRGBData(ParentActor, MoveTemp(DepthData));
            }
        }
    }
}

void URGBCameraComponent::RConfigure(
    const FRGBCameraConfig& Config, ARecorder* Recorder)
{ 
    FOV = Config.FOV;
    Width = Config.Width;
    Height = Config.Height;
    bOrthographic = Config.bOrthographic;
    OrthoWidth = Config.OrthoWidth;
    RGBRenderTarget->InitCustomFormat(Width, Height,
        PF_B8G8R8A8, false);
    RGBRenderTarget->UpdateResource();
    SetCaptureComponent();
    
    if (Config.RecordInterval > 0)
    {
        ParentActor = GetOwner();
        RecorderPtr = Recorder;
        RecordInterval = Config.RecordInterval;
        SetComponentTickEnabled(true);
        bRecorded = true;
    }
    else
    {
        SetComponentTickEnabled(false);
    }
    
    bBPConfigured = true;
}

void URGBCameraComponent::SetCaptureComponent() const
{
    if (CaptureComponent)
    {
        // Basic camera settings
        CaptureComponent->ProjectionType = bOrthographic ?
            ECameraProjectionMode::Orthographic : ECameraProjectionMode::Perspective;
        CaptureComponent->FOVAngle = FOV;
        CaptureComponent->OrthoWidth = OrthoWidth;

        FEngineShowFlags& ShowFlags = CaptureComponent->ShowFlags;
        ShowFlags.SetAtmosphere(true);
        ShowFlags.SetAntiAliasing(true);
        ShowFlags.SetDynamicShadows(true);
        ShowFlags.SetMotionBlur(false);
        ShowFlags.SetBloom(true);
        ShowFlags.SetAmbientOcclusion(true);
        ShowFlags.SetGlobalIllumination(true);
        ShowFlags.SetIndirectLightingCache(true);
        ShowFlags.SetTonemapper(true);
        ShowFlags.SetPostProcessing(true);
        ShowFlags.SetAmbientCubemap(true);

        // Capture settings
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        CaptureComponent->bCaptureEveryFrame = false;
        CaptureComponent->bCaptureOnMovement = false;
        CaptureComponent->bAlwaysPersistRenderingState = true;

        // Post Process Settings
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionIntensity = 1.0f;
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Enhanced indirect lighting
        CaptureComponent->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
        CaptureComponent->PostProcessSettings.IndirectLightingIntensity = 1.2f;
    }
    else 
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component not initialized!"));
    }
}

void URGBCameraComponent::InitializeRenderTargets()
{
    RGBRenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RGBRenderTarget->InitCustomFormat(Width, Height,
        PF_B8G8R8A8, false);
    
    RGBRenderTarget->UpdateResource();
    CaptureComponent->TextureTarget = RGBRenderTarget;
}

void URGBCameraComponent::CaptureRGBScene()
{
    if (CheckComponentAndRenderTarget())
    {
        return;
    }
    
    CaptureComponent->CaptureScene();
}

void URGBCameraComponent::AsyncGetRGBImageData(TFunction<void(const TArray<FColor>&)> Callback)
{
    AsyncTask(ENamedThreads::GameThread, [this, Callback = MoveTemp(Callback)]()
    {
        if (CheckComponentAndRenderTarget())
        {
            return;
        }
        
        CaptureComponent->CaptureScene();
        
        ProcessRGBTextureAsync([Callback](const TArray<FColor>& ColorData)
        {
            Callback(ColorData);
        });
    });
}

void URGBCameraComponent::ProcessRGBTextureAsyncRaw()
{
    FTextureRenderTargetResource* RenderTargetResource = 
        RGBRenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get render target resource!"));
        return;
    }

    // Ensure RGBData has correct size
    if (RGBData.Num() != Width * Height)
    {
        RGBData.SetNumUninitialized(Width * Height);
    }
    
    FReadSurfaceContext Context =
    {
        &RGBData,
        RenderTargetResource,
        FIntRect(0, 0, Width, Height),
        FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
    };

    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)(
        [Context](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.ReadSurfaceData(
                Context.RenderTarget->GetRenderTargetTexture(),
                Context.Rect,
                *Context.OutData,
                Context.Flags
            );
        });
}

void URGBCameraComponent::ProcessRGBTextureAsync(
    TFunction<void(const TArray<FColor>&)> OnComplete)
{
    FTextureRenderTargetResource* RenderTargetResource = 
        RGBRenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get render target resource!"));
        return;
    }

    // Ensure RGBData has correct size
    if (RGBData.Num() != Width * Height)
    {
        RGBData.SetNumUninitialized(Width * Height);
    }
    
    FReadSurfaceContext Context =
    {
        &RGBData,
        RenderTargetResource,
        FIntRect(0, 0, Width, Height),
        FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)
    };

    auto SharedCallback = MakeShared<
        TFunction<void(const TArray<FColor>&)>>(MoveTemp(OnComplete));
    // Capture the OnComplete callback in the render command
    ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)(
        [Context, SharedCallback](FRHICommandListImmediate& RHICmdList)
        {            
            RHICmdList.ReadSurfaceData(
                Context.RenderTarget->GetRenderTargetTexture(),
                Context.Rect,
                *Context.OutData,
                Context.Flags
            );
            
            (*SharedCallback)(*Context.OutData);
        });
}

bool URGBCameraComponent::CheckComponentAndRenderTarget() const
{
    if (!CaptureComponent || !RGBRenderTarget)
    {
        UE_LOG(LogTemp, Error, TEXT("URGBCameraComponent::CheckComponentAndRenderTarget: "
                                    "Capture component or render target not initialized!"));
        return true;
    }
    return false;
}

void URGBCameraComponent::CaptureRGBImageAsync()
{
    UE_LOG(LogTemp, Warning, TEXT("Starting async RGB capture"));
    
    if (CheckComponentAndRenderTarget())
    {
        return;
    }
    
    CaptureComponent->CaptureScene();
    
    ProcessRGBTextureAsync([this](const TArray<FColor>& ColorData)
    {
        OnRGBImageCaptured.Broadcast(ColorData);
    });
}