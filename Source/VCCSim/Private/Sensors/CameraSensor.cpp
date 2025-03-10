/*
* Copyright (C) 2025 Visual Computing Research Center, Shenzhen University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
    
    if (bRecorded && RecordState)
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
        RecordState = Recorder->RecordState;
        Recorder->OnRecordStateChanged.AddDynamic(this,
            &URGBCameraComponent::SetRecordState);
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

        // Change the capture source to HDR for better quality
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorHDR;
        
        FEngineShowFlags& ShowFlags = CaptureComponent->ShowFlags;
        ShowFlags.SetAtmosphere(true);
        ShowFlags.SetAntiAliasing(true);
        ShowFlags.SetDynamicShadows(true);
        // Disable motion blur for programmatically moved cameras
        ShowFlags.SetMotionBlur(false);
        ShowFlags.SetBloom(true);
        ShowFlags.SetAmbientOcclusion(true);
        ShowFlags.SetGlobalIllumination(true);
        ShowFlags.SetIndirectLightingCache(true);
        ShowFlags.SetTonemapper(true);
        ShowFlags.SetPostProcessing(true);
        ShowFlags.SetAmbientCubemap(true);

        // Capture settings
        CaptureComponent->bCaptureEveryFrame = false;
        CaptureComponent->bCaptureOnMovement = false;
        CaptureComponent->bAlwaysPersistRenderingState = true;

        // Enhanced post-processing settings compatible with UE 5.4
        
        // Ambient Occlusion
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionIntensity = 1.0f;
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        CaptureComponent->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        CaptureComponent->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Enhanced indirect lighting
        CaptureComponent->PostProcessSettings.bOverride_IndirectLightingIntensity = true;
        CaptureComponent->PostProcessSettings.IndirectLightingIntensity = 1.2f;
        
        // Enhanced color grading
        CaptureComponent->PostProcessSettings.bOverride_ColorGamma = true;
        CaptureComponent->PostProcessSettings.ColorGamma = FVector4(1.0f, 1.0f, 1.0f, 1.0f);
        CaptureComponent->PostProcessSettings.bOverride_ColorContrast = true;
        CaptureComponent->PostProcessSettings.ColorContrast = FVector4(1.05f, 1.05f, 1.05f, 1.0f);
        
        // Motion blur explicitly disabled for programmatically moved cameras
        CaptureComponent->PostProcessSettings.bOverride_MotionBlurAmount = true;
        CaptureComponent->PostProcessSettings.MotionBlurAmount = 0.0f;
        
        // Standard screen space reflections enhancements
        CaptureComponent->PostProcessSettings.bOverride_ScreenSpaceReflectionQuality = true;
        CaptureComponent->PostProcessSettings.ScreenSpaceReflectionQuality = 100.0f;
        CaptureComponent->PostProcessSettings.bOverride_ScreenSpaceReflectionIntensity = true; 
        CaptureComponent->PostProcessSettings.ScreenSpaceReflectionIntensity = 100.0f;
    }
    else 
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component not initialized!"));
    }
}

void URGBCameraComponent::InitializeRenderTargets()
{
    RGBRenderTarget = NewObject<UTextureRenderTarget2D>(this);
    // Use higher quality format with HDR support
    RGBRenderTarget->InitCustomFormat(Width, Height,
        PF_FloatRGBA, true);  
    
    // Enable MipMapping for better quality
    RGBRenderTarget->bAutoGenerateMips = true;
    
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

void URGBCameraComponent::AsyncGetRGBImageData(TFunction<void(const TArray<FLinearColor>&)> Callback)
{
    AsyncTask(ENamedThreads::GameThread, [this, Callback = MoveTemp(Callback)]()
    {
        if (CheckComponentAndRenderTarget())
        {
            return;
        }
        
        CaptureComponent->CaptureScene();
        
        ProcessRGBTextureAsync([Callback](const TArray<FLinearColor>& ColorData)
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
    TFunction<void(const TArray<FLinearColor>&)> OnComplete)
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
        TFunction<void(const TArray<FLinearColor>&)>>(MoveTemp(OnComplete));
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
    
    ProcessRGBTextureAsync([this](const TArray<FLinearColor>& ColorData)
    {
        OnRGBImageCaptured.Broadcast(ColorData);
    });
}