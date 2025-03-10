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

#include "Pawns/FlashPawn.h"
#include "Pawns/SimPath.h"

AFlashPawn::AFlashPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	FlashRoot = CreateDefaultSubobject<USceneComponent>(TEXT("FlashRoot"));
	RootComponent = FlashRoot;
}

void AFlashPawn::SetTarget(FVector Location, FRotator Rotation)
{
	TargetLocation = Location;
	TargetRotation = Rotation;
	bUseTarget = true;
}

void AFlashPawn::SetPath(
	const TArray<FVector>& Positions, const TArray<FRotator>& Rotations)
{
	if (Positions.Num() != Rotations.Num())
	{
		UE_LOG(LogTemp, Warning, TEXT("AFlashPawn::SetPath: "
			"Positions and Rotations must have the same length!"));
		return;
	}

	Path->SetNewTrajectory(Positions, Rotations);
	bUsePath = true;
	CurrentPathIndex = 0;
}

void AFlashPawn::BeginPlay()
{
	Super::BeginPlay();
	if (!Path)
	{
		Path = GetWorld()->SpawnActor<AVCCSimPath>();
	}
}

void AFlashPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (bUseTarget && bUsePath)
	{
		UE_LOG(LogTemp, Warning, TEXT("AFlashPawn::Tick: "
			"Both Target and Path are being used!"));
		return;
	}
	if (bUseTarget)
	{
		SetActorLocationAndRotation(TargetLocation, TargetRotation);
		bAcqReady = true;
		bUseTarget = false;
	}
	if (bUsePath && bMoveReady)
	{
		if (CurrentPathIndex < Path->GetNumberOfSplinePoints())
		{
			SetActorLocationAndRotation(
				Path->GetLocationAtSplinePoint(CurrentPathIndex),
				Path->GetRotationAtSplinePoint(CurrentPathIndex));
			CurrentPathIndex++;
			bAcqReady = true;
			bMoveReady = false;
		}
		else
		{
			bUsePath = false;
		}
	}
}
