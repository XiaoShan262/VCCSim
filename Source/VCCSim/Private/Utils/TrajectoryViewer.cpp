#include "Utils/TrajectoryViewer.h"
#include "Materials/MaterialInstanceDynamic.h"

UTrajectoryViewer::UTrajectoryViewer()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickInterval = 0.0333f;
}

void UTrajectoryViewer::BeginPlay()
{
    Super::BeginPlay();
    
    if (SplineComponent)
    {
        TotalLength = SplineComponent->GetSplineLength();
    }
    
    if (!PathMesh)
    {
        PathMesh = LoadObject<UStaticMesh>(nullptr,
            TEXT("/Engine/BasicShapes/Cylinder"));
    }

    const int32 EstimatedMeshCount = FMath::CeilToInt(DisplayDistance / StepSize);
    PathMeshes.Reserve(EstimatedMeshCount);
}

void UTrajectoryViewer::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    ClearSplineMeshes();
}

void UTrajectoryViewer::TickComponent(float DeltaTime, enum ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!SplineComponent || DisplayDistance <= 0.0f || !PathMesh || !PathMaterial)
    {
        return;
    }

    // Only update if the path has actually moved
    if (!FMath::IsNearlyEqual(LastTraveledDistance, TraveledDistance, 10))
    {
        UpdatePartialPath();
        LastTraveledDistance = TraveledDistance;
    }
}

void UTrajectoryViewer::UpdatePartialPath()
{
    const float ComponentZ = GetRelativeLocation().Z;
    const float EndDistance = FMath::Min(TraveledDistance + DisplayDistance, TotalLength);
    
    // Calculate segment counts
    const int32 RequiredMeshCount = FMath::CeilToInt((EndDistance - TraveledDistance) / StepSize);
    const int32 CurrentMeshCount = PathMeshes.Num();
    
    const float DistanceMoved = TraveledDistance - LastTraveledDistance;
    const int32 SegmentsToRemove = FMath::Max(0, FMath::FloorToInt(DistanceMoved / StepSize));
    const int32 SegmentsToAdd = RequiredMeshCount - (CurrentMeshCount - SegmentsToRemove);
    
    // Remove segments from front
    for (int32 i = 0; i < SegmentsToRemove && !PathMeshes.IsEmpty(); ++i)
    {
        if (PathMeshes[0])
        {
            PathMeshes[0]->DestroyComponent();
        }
        PathMeshes.PopFirst();
    }
    
    // Add new segments at the end
    float StartDistance = EndDistance - (SegmentsToAdd * StepSize);
    for (int32 i = 0; i < SegmentsToAdd; ++i)
    {
        float Distance = StartDistance + (i * StepSize);
        float NextDistance = FMath::Min(Distance + StepSize, EndDistance);
        
        FVector StartPos = GetPointAtDistance(Distance);
        StartPos.Z += ComponentZ;
        FVector StartTangent = SplineComponent->GetTangentAtDistanceAlongSpline(
            Distance, ESplineCoordinateSpace::World);
            
        FVector EndPos = GetPointAtDistance(NextDistance);
        EndPos.Z += ComponentZ;
        FVector EndTangent = SplineComponent->GetTangentAtDistanceAlongSpline(
            NextDistance, ESplineCoordinateSpace::World);

        USplineMeshComponent* SplineMesh = NewObject<USplineMeshComponent>(this);
        SplineMesh->SetMobility(EComponentMobility::Movable);
        SplineMesh->SetVisibility(true);
        SplineMesh->SetHiddenInGame(false);
        SplineMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        SplineMesh->SetStaticMesh(PathMesh);
        SplineMesh->SetMaterial(0, PathMaterial);
        SplineMesh->SetForwardAxis(ESplineMeshAxis::X);
        SplineMesh->SetUsingAbsoluteLocation(true);
        SplineMesh->SetUsingAbsoluteRotation(true);
        
        SplineMesh->SetStartAndEnd(StartPos, StartTangent, EndPos, EndTangent);   
        SplineMesh->SetStartScale(FVector2D(PathWidth, PathWidth));
        SplineMesh->SetEndScale(FVector2D(PathWidth, PathWidth));
        
        SplineMesh->RegisterComponent();
        SplineMesh->AttachToComponent(this, 
            FAttachmentTransformRules::KeepWorldTransform);
        
        PathMeshes.PushLast(SplineMesh);
    }
}

void UTrajectoryViewer::ClearSplineMeshes()
{
    while (!PathMeshes.IsEmpty())
    {
        if (PathMeshes[0])
        {
            PathMeshes[0]->DestroyComponent();
        }
        PathMeshes.PopFirst();
    }
}


FVector UTrajectoryViewer::GetPointAtDistance(float Distance) const
{
    return SplineComponent ? SplineComponent->GetLocationAtDistanceAlongSpline(
        Distance, ESplineCoordinateSpace::World) : FVector::ZeroVector;
}