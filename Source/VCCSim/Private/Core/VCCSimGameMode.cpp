#include "Core/VCCSimGameMode.h"
#include "Core/VCCHUD.h"
#include "IImageWrapperModule.h"


ARatSimGameMode::ARatSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
	DefaultPawnClass = nullptr;
	HUDClass = ARatHUD::StaticClass();
}


void ARatSimGameMode::StartPlay()
{
    Super::StartPlay();
    UserSettings = GEngine->GetGameUserSettings();
}