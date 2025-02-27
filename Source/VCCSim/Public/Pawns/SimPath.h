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
