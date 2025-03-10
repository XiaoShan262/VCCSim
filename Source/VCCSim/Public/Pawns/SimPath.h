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
#include "GameFramework/Actor.h"

#include "SimPath.generated.h"

class USplineComponent;
struct FSplinePoint;

UCLASS(HideCategories=("Default", "Replication", "REndering", "Collision",
	"HLOD", "Input", "Physics", "Networking", "Actor", "Cooking", "Hidden",
	"World Partition", "Tick", "Events", "Data Layers"))
class VCCSIM_API AVCCSimPath : public AActor
{
	GENERATED_BODY()
public:	
	AVCCSimPath();
	virtual void OnConstruction(const FTransform& Transform) override; 

	UFUNCTION(BlueprintCallable, Category="Default")
	void DiscoverTraceIgnores();

	UFUNCTION(BlueprintCallable)
	void SnapAllPointsToGround();

	UFUNCTION(BlueprintCallable)
	void ReverseSpline();
	
	UFUNCTION(BlueprintCallable)
	void FlattenAllTangents();
	
	UFUNCTION(BlueprintCallable)
	void MovePivotToFirstPoint();

	UFUNCTION(BlueprintCallable)
	void SetNewTrajectory(const TArray<FVector>& Positions,
		const TArray<FRotator>& Rotations);

	int32 GetNumberOfSplinePoints() const;
	FVector GetLocationAtSplinePoint(int32 Index) const;
	FRotator GetRotationAtSplinePoint(int32 Index) const;
	
public:
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Default")
	TObjectPtr<USplineComponent> Spline;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Path")
	bool SnapToGround;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Path")
	bool bReverseSpline;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Path")
	bool ClosedSpline;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Path")
	bool FlattenTangents;
	
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Hidden")
	TArray<FSplinePoint> SplinePoints;
	UPROPERTY(BlueprintReadWrite, EditInstanceOnly, Category="Hidden")
	TArray<AActor*> ExcludedInTrace;
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Hidden")
	TMap<int32,FVector> NewSplinePointLocations;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category="Read Only")
	float PathLength;

	TArray<FVector> PendingPositions;
	TArray<FRotator> PendingRotations;
	bool bIsProcessingTrajectory = false;
	int32 ProcessedPoints = 0; // New variable to track progress
	FTimerHandle TrajectoryTimerHandle;
    
	void ProcessPendingTrajectory();
};
