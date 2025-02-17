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

void URGBCameraComponent::RGBConfigure(const RGBCameraConfig& Config)
{ 
    FOV = Config.FOV;
    Width = Config.Width;
    Height = Config.Height;
    bOrthographic = Config.bOrthographic;
    OrthoWidth = Config.OrthoWidth;
    CaptureRate = 1.f / Config.CaptureRate;
    SetCaptureComponent();
    PrimaryComponentTick.bCanEverTick = false;

    
}

void URGBCameraComponent::SetCaptureComponent() const
{
    if (CaptureComponent)
    {
        CaptureComponent->ProjectionType = bOrthographic ?
            ECameraProjectionMode::Orthographic : ECameraProjectionMode::Perspective;
        CaptureComponent->FOVAngle = FOV;
        CaptureComponent->OrthoWidth = OrthoWidth;
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        CaptureComponent->bCaptureEveryFrame = false;
        CaptureComponent->bCaptureOnMovement = true;

        FEngineShowFlags& ShowFlags = CaptureComponent->ShowFlags;
        ShowFlags.SetAntiAliasing(true);
        ShowFlags.SetDynamicShadows(true);
        ShowFlags.SetMotionBlur(false);
    }
    else 
    {
        UE_LOG(LogTemp, Error, TEXT("Capture component not initialized!"));
    }
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

void URGBCameraComponent::InitializeRenderTargets()
{
    RGBRenderTarget = NewObject<UTextureRenderTarget2D>(this);
    RGBRenderTarget->InitCustomFormat(Width, Height,
        PF_B8G8R8A8, true);
    
    RGBRenderTarget->UpdateResource();
    CaptureComponent->TextureTarget = RGBRenderTarget;
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

TArray<FRGBPixel> URGBCameraComponent::GetRGBImageDataGameThread()
{
    check(IsInGameThread());
    if (CheckComponentAndRenderTarget()) return {};

    CaptureComponent->CaptureScene();
    ProcessRGBTexture();
    
    return GetRGBImage();
}

void URGBCameraComponent::CaptureRGBScene()
{
    if (CheckComponentAndRenderTarget())
    {
        UE_LOG(LogTemp, Error, TEXT("URGBCameraComponent: "
                                    "Capture component or render target not initialized!"));
        return;
    }
    
    CaptureComponent->CaptureScene();
    ProcessRGBTexture();
}

void URGBCameraComponent::AsyncGetRGBImageData(
    TFunction<void(TArray<FRGBPixel>)> Callback)
{
    AsyncTask(ENamedThreads::GameThread, [this, Callback]() {
        TArray<FRGBPixel> ImageData = GetRGBImageDataGameThread();
        Callback(ImageData);
    });
}

void URGBCameraComponent::ProcessRGBTexture()
{
    FTextureRenderTargetResource* RenderTargetResource =
        RGBRenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to get render target resource!"));
        return;
    }

    RGBData.Empty(Width * Height);
    
    struct FReadSurfaceContext
    {
        TArray<FColor>* OutData;
        FTextureRenderTargetResource* RenderTarget;
        FIntRect Rect;
        FReadSurfaceDataFlags Flags;
    };

    FReadSurfaceContext Context = {
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

    FlushRenderingCommands();
}

TArray<FRGBPixel> URGBCameraComponent::GetRGBImage()
{
    TArray<FRGBPixel> RGBImage;
    RGBImage.Empty(Width * Height);

    if (RGBData.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No RGB data available!"));
        return RGBImage;
    }

    for (const FColor& Color : RGBData)
    {
        RGBImage.Add(FRGBPixel(Color.R, Color.G, Color.B));
    }

    return RGBImage;
}