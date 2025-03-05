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

#include "Pawns/DronePawn.h"
#include "EnhancedInputSubsystems.h"
#include "Components/SplineComponent.h"
#include "Pawns/SimPath.h"

void ADronePawn::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    DistanceTraveled = 0.f;
    CourseDistance = 0.f;
    LastCourseDistance = 0.f; 
    Laps = 1.f;
    CalculateDistance();
    FollowThePathAndSteer(1);
    
}
void ADronePawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    CalculateDistance();
    FollowThePathAndSteer(DeltaSeconds);
    AutoMove(DeltaSeconds);
}
void ADronePawn::AddMapContext()
{
    if (APlayerController* PlayerController = Cast<APlayerController>(Controller))
    {
        if (UEnhancedInputLocalPlayerSubsystem* Subsystem = ULocalPlayer::GetSubsystem<
            UEnhancedInputLocalPlayerSubsystem>(PlayerController->GetLocalPlayer()))
        {
            Subsystem->AddMappingContext(DroneInputMappingContext, 0);
        }
    }
}

void ADronePawn::SetTarget(FVector Location, FRotator Rotation)
{
    TargetLocation = Location;
    TargetRotation = Rotation;
    bUseTarget = true;
}

bool ADronePawn::IfCloseToTarget(FVector Location, FRotator Rotation) const
{
    // Calculate position error
    const FVector CurrentLocation = GetActorLocation();
    const FRotator CurrentRotation = GetActorRotation();
    const FVector PositionError = Location - CurrentLocation;
    
    // Calculate rotation error
    const FRotator RotationError = Rotation - CurrentRotation;
    
    return PositionError.Size() < PositionThreshold &&
           RotationError.Vector().Size() < RotationThreshold;
}
void ADronePawn::CalculateDistance()
{
    if (Path)
    {
        Laps = FMath::Floor(DistanceTraveled / Path->Spline->GetSplineLength());
        CourseDistance = DistanceTraveled - Laps * Path->Spline->GetSplineLength();
    }
    else
    {
        CourseDistance = DistanceTraveled;
    }
}

void ADronePawn::FollowThePathAndSteer(float DeltaTime)
{
    if (Path)
    {
        const auto NewLocation = Path->Spline->GetLocationAtDistanceAlongSpline(
            CourseDistance, ESplineCoordinateSpace::World);
        const auto NextRotation = Path->Spline->GetRotationAtDistanceAlongSpline(
            CourseDistance, ESplineCoordinateSpace::World);
        const auto NewRotation = FMath::RInterpTo(GetActorRotation(), NextRotation,
            DeltaTime, 8.f);
        SetActorLocationAndRotation(NewLocation,
                FRotator(0, NewRotation.Yaw, 0));
    }
}

void ADronePawn::AutoMove(double DeltaSeconds)
{
    if (IfAutoMove)
    {
        DistanceTraveled += DeltaSeconds * 100;
    }
}
