#include "Sensors/CameraSensor.h"

#include "RenderingThread.h"
#include "Async/AsyncWork.h"
#include "Windows/WindowsHWrapper.h"

URGBCameraComponent::URGBCameraComponent()
    : FOV(90.0f)
    , Width(512)
    , Height(512)
    , bOrthographic(false)
    , OrthoWidth(512.0f)
    , bAutoCapture(false)
    , CaptureRate(1.f / 30.f)
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
    
    if (bAutoCapture)
    {
        TimeSinceLastCapture += DeltaTime;
        if (TimeSinceLastCapture >= CaptureRate)
        {
            CaptureRGBScene();
            TimeSinceLastCapture = 0.0f;
        }
    }
}

void URGBCameraComponent::RGBConfigure(const FRGBCameraConfig& Config)
{ 
    FOV = Config.FOV;
    Width = Config.Width;
    Height = Config.Height;
    bOrthographic = Config.bOrthographic;
    OrthoWidth = Config.OrthoWidth;
    CaptureRate = 1.f / Config.CaptureRate;
    RGBRenderTarget->InitCustomFormat(Width, Height,
        PF_B8G8R8A8, false);
    RGBRenderTarget->UpdateResource();
    SetCaptureComponent();
    PrimaryComponentTick.bCanEverTick = false;

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

void URGBCameraComponent::AsyncGetRGBImageData()
{
    AsyncTask(ENamedThreads::GameThread, [this]()
    {
        CaptureRGBImageAsync();
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

    auto SharedCallback = MakeShared<TFunction<void(const TArray<FColor>&)>>(MoveTemp(OnComplete));
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
            
            // Execute callback on game thread when render is complete
            AsyncTask(ENamedThreads::GameThread,
                [SharedCallback, OutData = Context.OutData]()
                {
                    (*SharedCallback)(*OutData);
                }
            );
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
        TArray<FRGBPixel> RGBData = TransformFColorToRGBPixel(ColorData);
        OnRGBImageCaptured.Broadcast(RGBData);
    });
}

TArray<FRGBPixel> TransformFColorToRGBPixel(const TArray<FColor>& Colors)
{
    TArray<FRGBPixel> RGBImage;
    RGBImage.Init(FRGBPixel(), Colors.Num());
    ParallelFor(Colors.Num(), [&](int32 i)
        {
            RGBImage[i].R = Colors[i].R;
            RGBImage[i].G = Colors[i].G;
            RGBImage[i].B = Colors[i].B;
        });
    return RGBImage;
}
