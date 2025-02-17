#include "Core/VCCSimGameInstance.h"
#include "Kismet/GameplayStatics.h"

URatSimGameInstance::URatSimGameInstance()
{
	// Default values
	MainMenuMapName = TEXT("MainMenu");
	SimulationSpeed = 1.0f;
}

void URatSimGameInstance::Init()
{
	Super::Init();

	// Initialize your game instance here
	UE_LOG(LogTemp, Log, TEXT("RatSim GameInstance Initialized"));
}

void URatSimGameInstance::LoadMap(const FString& MapName)
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

void URatSimGameInstance::ReturnToMainMenu()
{
	UGameplayStatics::OpenLevel(this, *MainMenuMapName);
}

void URatSimGameInstance::SaveGameState()
{
}

void URatSimGameInstance::LoadGameState()
{
}

void URatSimGameInstance::SetSimulationSpeed(float Speed)
{
	SimulationSpeed = FMath::Clamp(Speed, 0.1f, 10.0f);
}