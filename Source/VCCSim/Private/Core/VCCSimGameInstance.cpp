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