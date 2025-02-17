#include "Core/VCCSimGameInstance.h"
#include "Kismet/GameplayStatics.h"

UVCCSimGameInstance::UVCCSimGameInstance()
{
	// Default values
	MainMenuMapName = TEXT("MainMenu");
	SimulationSpeed = 1.0f;
}

void UVCCSimGameInstance::Init()
{
	Super::Init();

	// Initialize your game instance here
	UE_LOG(LogTemp, Log, TEXT("VCCSim GameInstance Initialized"));
}

void UVCCSimGameInstance::LoadMap(const FString& MapName)
{
	// Check if the map name is valid
	if (AvailableMaps.Contains(MapName))
	{
		UGameplayStatics::OpenLevel(this, *MapName);
	}
	else
	{
		UE_LOG(LogTemp, Warning,
			TEXT("Attempted to load invalid map: %s"), *MapName);
	}
}

void UVCCSimGameInstance::ReturnToMainMenu()
{
	UGameplayStatics::OpenLevel(this, *MainMenuMapName);
}

void UVCCSimGameInstance::SaveGameState()
{
}

void UVCCSimGameInstance::LoadGameState()
{
}

void UVCCSimGameInstance::SetSimulationSpeed(float Speed)
{
	SimulationSpeed = FMath::Clamp(Speed, 0.1f, 10.0f);
}