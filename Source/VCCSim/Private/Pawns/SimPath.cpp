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
				Spline->SetLocationAtSplinePoint(i, HitResult.Location, ESplineCoordinateSpace::World);
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

void AVCCSimPath::SetNewTrajectory(
	const TArray<FVector>& Positions, const TArray<FRotator>& Rotations)
{
	Spline->ClearSplinePoints(false);
	for (int32 i = 0; i < Positions.Num(); i++)
	{
		Spline->AddSplinePoint(Positions[i], ESplineCoordinateSpace::World);
		Spline->SetRotationAtSplinePoint(i, Rotations[i], ESplineCoordinateSpace::World);
	}
	Spline->UpdateSpline();
	PathLength = Spline->GetSplineLength();

	MovePivotToFirstPoint();
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