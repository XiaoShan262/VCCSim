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

#include "Pawns/SimPath.h"
#include "Components/SplineComponent.h"
#include "Kismet/GameplayStatics.h"

AVCCSimPath::AVCCSimPath()
{
	Spline = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));
	RootComponent = Spline;
	
	SnapToGround = false;
	bReverseSpline = false;
	ClosedSpline = false;
	FlattenTangents = false;
	
	PathLength = 0;
}

void AVCCSimPath::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	PathLength = Spline->GetSplineLength();
	DiscoverTraceIgnores();
	SnapAllPointsToGround();
	ReverseSpline();
	FlattenAllTangents();
	if (ClosedSpline)
	{
		Spline->SetClosedLoop(true);
	}
	MovePivotToFirstPoint();
}

void AVCCSimPath::DiscoverTraceIgnores()
{
	// Find all actors which has Tag IgnoreTrace.
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsWithTag(GetWorld(), "IgnoreTrace", FoundActors);
	for (AActor* Actor : FoundActors)
	{
		ExcludedInTrace.Add(Actor);
	}
}

void AVCCSimPath::SnapAllPointsToGround()
{
	if (SnapToGround)
	{
		for (int32 i = 0; i < Spline->GetNumberOfSplinePoints(); i++)
		{
			FVector Location = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
			FHitResult HitResult;
			FVector Start = Location + FVector(0, 0, 150);
			FVector End = Location - FVector(0, 0, 10000);
			FCollisionQueryParams TraceParams;
			TraceParams.AddIgnoredActors(ExcludedInTrace);
			if (GetWorld()->LineTraceSingleByChannel(HitResult, Start, End,
				ECollisionChannel::ECC_Visibility, TraceParams))
			{
				Spline->SetLocationAtSplinePoint(i, HitResult.Location,
					ESplineCoordinateSpace::World);
			}
		}
		Spline->UpdateSpline();
		SnapToGround = false;
	}
}

void AVCCSimPath::ReverseSpline()
{
	if (bReverseSpline)
	{
		SplinePoints.Empty();
		// Store the points in SplinePoints
		for (int32 i = 0; i < Spline->GetNumberOfSplinePoints(); i++)
		{
			FSplinePoint Point;
			Point.Position = Spline->GetLocationAtSplinePoint(i, ESplineCoordinateSpace::World);
			Point.Rotation = Spline->GetRotationAtSplinePoint(i, ESplineCoordinateSpace::World);
			Point.Scale = Spline->GetScaleAtSplinePoint(i);
			SplinePoints.Add(Point);
		}
		// Recreate the spline with the points in reverse order
		Spline->ClearSplinePoints();
		for (int32 i = SplinePoints.Num() - 1; i >= 0; i--)
		{
			Spline->AddSplinePoint(SplinePoints[i].Position,
				ESplineCoordinateSpace::World, false);
			Spline->SetSplinePointType(i, SplinePoints[i].Type, false);
			Spline->SetTangentAtSplinePoint(i, SplinePoints[i].ArriveTangent,
				ESplineCoordinateSpace::World, false);
			Spline->SetTangentAtSplinePoint(i, SplinePoints[i].LeaveTangent,
				ESplineCoordinateSpace::World, false);
		}
		Spline->UpdateSpline();
		bReverseSpline = false;
	}
}

void AVCCSimPath::FlattenAllTangents()
{
	if (FlattenTangents)
	{
		for (int32 i = 0; i < Spline->GetNumberOfSplinePoints(); i++)
		{
			const auto ArriveTangent = Spline->GetArriveTangentAtSplinePoint(
			i, ESplineCoordinateSpace::Local);
			Spline->SetTangentAtSplinePoint(i,
				{ArriveTangent.X, ArriveTangent.Y, 0.f},
				ESplineCoordinateSpace::Local);
		}
		Spline->UpdateSpline();
		FlattenTangents = false;
	}
}

void AVCCSimPath::MovePivotToFirstPoint()
{
	NewSplinePointLocations.Empty();
	// Save the locations of all points
	for (int32 i = 0; i < Spline->GetNumberOfSplinePoints(); i++)
	{
		NewSplinePointLocations.Add(i, Spline->GetLocationAtSplinePoint(i,
			ESplineCoordinateSpace::World));
	}
	// Move the Actor to the First Point and Relocate All Points
	SetActorLocation(NewSplinePointLocations[0]);
	for (int32 i = 0; i < Spline->GetNumberOfSplinePoints(); i++)
	{
		Spline->SetLocationAtSplinePoint(i, NewSplinePointLocations[i],
			ESplineCoordinateSpace::World);
	}
	Spline->UpdateSpline();
}

int32 AVCCSimPath::GetNumberOfSplinePoints() const
{
	return Spline->GetNumberOfSplinePoints();
}

FVector AVCCSimPath::GetLocationAtSplinePoint(int32 Index) const
{
	return Spline->GetLocationAtSplinePoint(Index, ESplineCoordinateSpace::World);
}

FRotator AVCCSimPath::GetRotationAtSplinePoint(int32 Index) const
{
	return Spline->GetRotationAtSplinePoint(Index, ESplineCoordinateSpace::World);
}

void AVCCSimPath::SetNewTrajectory(
	const TArray<FVector>& Positions, const TArray<FRotator>& Rotations)
{
    // Store the data for deferred processing
    PendingPositions = Positions;
    PendingRotations = Rotations;
    
    // If not already processing, start the process
    if (!bIsProcessingTrajectory)
    {
        bIsProcessingTrajectory = true;
        ProcessedPoints = 0; // Reset the counter
        
        // Clear existing points before starting
        Spline->ClearSplinePoints(true);
        
        // Use AsyncTask to ensure we run on the game thread
        AsyncTask(ENamedThreads::GameThread, [this]()
        {
            ProcessPendingTrajectory();
        });
    }
}

void AVCCSimPath::ProcessPendingTrajectory()
{
    // Process points in batches
    const int32 BatchSize = 100; // Adjust based on your needs
    const int32 TotalPoints = FMath::Min(PendingPositions.Num(), PendingRotations.Num());
	

    if (ProcessedPoints < TotalPoints)
    {
        const int32 PointsToProcess = FMath::Min(BatchSize, TotalPoints - ProcessedPoints);
        
        for (int32 i = 0; i < PointsToProcess; i++)
        {
            const int32 Index = ProcessedPoints + i;
            const int32 SplineIndex = ProcessedPoints + i; // This will be the spline point index

            Spline->AddSplinePoint(PendingPositions[Index],
                ESplineCoordinateSpace::World, false);
            Spline->SetRotationAtSplinePoint(SplineIndex,
                PendingRotations[Index], ESplineCoordinateSpace::World, false);
			// Setting Point Type, default type is linear, so it must be set "Curve" to make the path smoother.
			Spline->SetSplinePointType(SplineIndex, ESplinePointType::Curve, false);
        }
        
        ProcessedPoints += PointsToProcess;
        
        // If we have more points to process, schedule the next batch
        if (ProcessedPoints < TotalPoints)
        {
            GetWorldTimerManager().SetTimerForNextTick(this,
                &AVCCSimPath::ProcessPendingTrajectory);
            return;
        }
    }
    
    // Finalize the spline
    Spline->UpdateSpline();
    PathLength = Spline->GetSplineLength();
    
    // We're done processing
    bIsProcessingTrajectory = false;
    PendingPositions.Empty();
    PendingRotations.Empty();
}