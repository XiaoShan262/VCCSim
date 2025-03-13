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

#pragma once
#include "CoreMinimal.h"
#include "SafeZoneManager.generated.h"

UCLASS()
class VCCSIM_API USafeZoneManager : public UObject
{
	GENERATED_BODY()

public:
	// Initialize the system with base environment SDF
	void Initialize();

	// For GRPC Calls
	bool IsPositionSafeDrone(const FVector& Position);
	bool IsPositionSafeCar(const FVector& Position);
	float GetSafetyMargin(const FVector& Position);
    
	// Update system with dynamic actors
	void UpdateDynamicObstacles(TArray<AActor*> DynamicActors);

	float SafeDistanceDrone = 500.f;
	float SafeDistanceCar = 1000.f;
	float SafeHeight = 500.f;
    
private:
	// Cache the world distance field
	FDistanceFieldVolumeData* WorldDistanceField;
    
	// Dynamic obstacle distance field
	TSharedPtr<FDistanceFieldVolumeData> DynamicDistanceField;
    
	// Update frequency for dynamic objects
	float UpdateFrequency = 0.1f; // 10 updates per second
    
	// Thread-safe position checking
	FCriticalSection SafetyCheckLock;
};
