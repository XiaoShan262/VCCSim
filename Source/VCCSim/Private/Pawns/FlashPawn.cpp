// MIT License
// 
// Copyright (c) 2025 Mingyang Wang
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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
