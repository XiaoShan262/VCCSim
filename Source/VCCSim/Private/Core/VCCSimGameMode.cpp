#include "Core/VCCSimGameMode.h"
#include "Core/VCCHUD.h"
#include "IImageWrapperModule.h"


AVCCSimGameMode::AVCCSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
	DefaultPawnClass = nullptr;
	HUDClass = AVCCHUD::StaticClass();
}


void AVCCSimGameMode::StartPlay()
{
    Super::StartPlay();
    UserSettings = GEngine->GetGameUserSettings();
}