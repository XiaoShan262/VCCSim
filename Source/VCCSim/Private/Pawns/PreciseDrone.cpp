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

#include "Pawns/PreciseDrone.h"
#include "Simulation/Recorder.h"
#include "Components/BoxComponent.h"
#include "Components/SplineComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputMappingContext.h"
#include "InputAction.h"
#include "EntitySystem/MovieSceneEntitySystemRunner.h"

APreciseDrone::APreciseDrone()
{
    PrimaryActorTick.bCanEverTick = true;

    // Create root component
    DroneRoot = CreateDefaultSubobject<UBoxComponent>(TEXT("DroneRoot"));
    RootComponent = DroneRoot;
    
    // Create drone mesh
    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(DroneRoot);
    DroneMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    
    // Create rotor meshes directly attached to the drone root
    RotorMesh1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh1"));
    RotorMesh2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh2"));
    RotorMesh3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh3"));
    RotorMesh4 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh4"));
    
    RotorMesh1->SetupAttachment(DroneRoot);
    RotorMesh2->SetupAttachment(DroneRoot);
    RotorMesh3->SetupAttachment(DroneRoot);
    RotorMesh4->SetupAttachment(DroneRoot);

    RotorMesh1->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    RotorMesh2->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    RotorMesh3->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    RotorMesh4->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    
    // Hide from scene capture by default
    DroneMesh->bHiddenInSceneCapture = true;
    RotorMesh1->bHiddenInSceneCapture = true;
    RotorMesh2->bHiddenInSceneCapture = true;
    RotorMesh3->bHiddenInSceneCapture = true;
    RotorMesh4->bHiddenInSceneCapture = true;
}

void APreciseDrone::BeginPlay()
{
    Super::BeginPlay();
    AddMapContext();
}

void APreciseDrone::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
}

void APreciseDrone::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Handle recording if enabled
    if (bRecorded && RecordState)
    {
        TimeSinceLastCapture += DeltaTime;
        if (TimeSinceLastCapture >= RecordInterval)
        {
            TimeSinceLastCapture = 0.0f;
            FPoseData PoseData;
            PoseData.Location = GetActorLocation();
            PoseData.Rotation = GetActorRotation();
            PoseData.Timestamp = FPlatformTime::Seconds();
            Recorder->SubmitPoseData(this, MoveTemp(PoseData));
        }
    }

    // Process movement inputs or target movement
    if (bUseTarget)
    {
        MoveToTarget(DeltaTime);
    }
    else if (bUsePath)
    {
        if (CourseDistance > LastCourseDistance)
        {
            FollowThePathAndSteer();
        }
        else
        {
            bUsePath = false;
        }
    }
    else
    {
        if (ManualControl)
        {
            // Apply manual input controls with physics-based movement
            FVector CurrentLocation = GetActorLocation();
            FRotator CurrentRotation = GetActorRotation();
            
            // Calculate desired vertical speed
            float DesiredVerticalSpeed = VerticalInput * MaxVerticalSpeed;
            
            // Calculate movement direction with rotation-based direction
            FRotator YawRotation(0.0f, CurrentRotation.Yaw, 0.0f);
            FVector ForwardDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
            FVector RightDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
            
            // Calculate desired horizontal movement
            FVector DesiredHorizontalDir = (ForwardDirection * MovementInput.Y + RightDirection * MovementInput.X);
            float DesiredHorizontalSpeed = MaxHorizontalSpeed;
            if (DesiredHorizontalDir.SizeSquared() > 0.0f)
            {
                DesiredHorizontalDir.Normalize();
                DesiredHorizontalSpeed = MovementInput.Size() * MaxHorizontalSpeed;
            }
            else
            {
                DesiredHorizontalSpeed = 0.0f;
            }
            
            // Apply acceleration limits
            CurrentVerticalSpeed = FMath::FInterpTo(
                CurrentVerticalSpeed, DesiredVerticalSpeed, 
                DeltaTime, VerticalAcceleration / 100.0f);
                
            CurrentHorizontalSpeed = FMath::FInterpTo(
                CurrentHorizontalSpeed, DesiredHorizontalSpeed, 
                DeltaTime, HorizontalAcceleration / 100.0f);
                
            // Apply total thrust limitation
            float TotalThrust = FMath::Sqrt(
                FMath::Square(CurrentHorizontalSpeed / MaxHorizontalSpeed) + 
                FMath::Square(CurrentVerticalSpeed / MaxVerticalSpeed));
            
            if (TotalThrust > 1.0f)
            {
                CurrentHorizontalSpeed /= TotalThrust;
                CurrentVerticalSpeed /= TotalThrust;
            }
            
            // Calculate and apply movement
            FVector HorizontalMovement = DesiredHorizontalDir * CurrentHorizontalSpeed * DeltaTime;
            FVector VerticalMovement(0, 0, CurrentVerticalSpeed * DeltaTime);
            CurrentLocation += HorizontalMovement + VerticalMovement;
            
            // Handle yaw rotation
            CurrentRotation.Yaw += YawInput * YawAcceleration * DeltaTime;
            
            // Calculate pitch and roll based on movement
            float TargetPitch = -MaxPitch * MovementInput.Y * (CurrentHorizontalSpeed / MaxHorizontalSpeed);
            float TargetRoll = MaxRoll * MovementInput.X * (CurrentHorizontalSpeed / MaxHorizontalSpeed);
            
            // Apply smooth pitch and roll changes
            CurrentRotation.Pitch = FMath::FInterpTo(
                CurrentRotation.Pitch, TargetPitch, 
                DeltaTime, PitchRollAcceleration);
            
            CurrentRotation.Roll = FMath::FInterpTo(
                CurrentRotation.Roll, TargetRoll, 
                DeltaTime, PitchRollAcceleration);
            
            // Apply the new position and rotation
            SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
        }
    }
    
    // Always rotate the rotors
    RotateRotors(DeltaTime);
}

void APreciseDrone::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    EnhancedInputComponent = CastChecked<UEnhancedInputComponent>(PlayerInputComponent);

    if (EnhancedInputComponent)
    {
        EnhancedInputComponent->BindAction(ThrottleAction, ETriggerEvent::Triggered,
            this, &APreciseDrone::HandleThrottleInput);
        EnhancedInputComponent->BindAction(ThrottleAction, ETriggerEvent::Completed,
            this, &APreciseDrone::HandleThrottleReleased);
        
        EnhancedInputComponent->BindAction(MovementAction, ETriggerEvent::Triggered,
            this, &APreciseDrone::HandleMovementInput);
        EnhancedInputComponent->BindAction(MovementAction, ETriggerEvent::Completed,
            this, &APreciseDrone::HandleMovementReleased);
        
        EnhancedInputComponent->BindAction(YawAction, ETriggerEvent::Triggered,
            this, &APreciseDrone::HandleYawInput);
        EnhancedInputComponent->BindAction(YawAction, ETriggerEvent::Completed,
            this, &APreciseDrone::HandleYawReleased);

        EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered,
            this, &APreciseDrone::HandleLookInput);
    }
}

void APreciseDrone::FollowThePathAndSteer()
{
    if (!Path || !Path->Spline)
    {
        bUsePath = false;
        return;
    }
    
    float DeltaTime = GetWorld()->GetDeltaSeconds();
    
    // Calculate desired speeds based on curvature of path ahead
    float SplineLength = Path->Spline->GetSplineLength();
    float LookAheadDistance = FMath::Clamp(CurrentHorizontalSpeed * 0.3f, 50.0f, 150.0f);
    float NextDistance = FMath::Min(CourseDistance + LookAheadDistance, SplineLength);
    
    // Get current and look-ahead positions/directions for curvature calculation
    FVector CurrentSplinePos = Path->Spline->GetLocationAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World);
    FVector LookAheadPos = Path->Spline->GetLocationAtDistanceAlongSpline(
        NextDistance, ESplineCoordinateSpace::World);
    FVector CurrentTangent = Path->Spline->GetTangentAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World).GetSafeNormal();
    FVector NextTangent = Path->Spline->GetTangentAtDistanceAlongSpline(
        NextDistance, ESplineCoordinateSpace::World).GetSafeNormal();
    
    // Calculate curvature (approximate)
    float TangentDot = FVector::DotProduct(CurrentTangent, NextTangent);
    float CurvatureFactor = FMath::Clamp(TangentDot, 0.5f, 1.0f); // 1.0 = straight, 0.5 = sharp curve
    
    // Adjust speed based on curvature (slow down for curves)
    float DesiredHorizontalSpeed = MaxHorizontalSpeed * CurvatureFactor;
    
    // Get vertical component of upcoming path
    float VerticalDelta = LookAheadPos.Z - CurrentSplinePos.Z;
    float DesiredVerticalSpeed = FMath::Clamp(VerticalDelta / LookAheadDistance * MaxVerticalSpeed, 
                                             -MaxVerticalSpeed, MaxVerticalSpeed);
    
    // Apply acceleration limits
    CurrentHorizontalSpeed = FMath::FInterpTo(
        CurrentHorizontalSpeed, DesiredHorizontalSpeed, 
        DeltaTime, HorizontalAcceleration / 50.0f); // Increased responsiveness
    
    CurrentVerticalSpeed = FMath::FInterpTo(
        CurrentVerticalSpeed, DesiredVerticalSpeed, 
        DeltaTime, VerticalAcceleration / 50.0f);  // Increased responsiveness
    
    // Apply total thrust limitation
    float TotalThrust = FMath::Sqrt(
        FMath::Square(CurrentHorizontalSpeed / MaxHorizontalSpeed) + 
        FMath::Square(CurrentVerticalSpeed / MaxVerticalSpeed));
    
    if (TotalThrust > 1.0f)
    {
        CurrentHorizontalSpeed /= TotalThrust;
        CurrentVerticalSpeed /= TotalThrust;
    }
    
    // Calculate how far to advance along the spline this frame
    float SpeedAlongSpline = FMath::Sqrt(FMath::Square(CurrentHorizontalSpeed) + FMath::Square(CurrentVerticalSpeed));
    float DistanceToAdvance = SpeedAlongSpline * DeltaTime;
    
    // Update distance traveled along path
    DistanceTraveled += DistanceToAdvance;
    
    // Update CourseDistance
    LastCourseDistance = CourseDistance;
    CourseDistance += DistanceToAdvance;
    
    // Ensure we stay within spline bounds (for looping)
    while (CourseDistance >= SplineLength)
    {
        CourseDistance -= SplineLength;
    }
    
    // Get exact position and rotation from the spline
    FVector TargetPosition = Path->Spline->GetLocationAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World);
    FRotator SplineRotation = Path->Spline->GetRotationAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World);
    
    // Get current rotation
    FRotator CurrentRotation = GetActorRotation();
    
    // Get forward direction from spline for realistic pitch/roll
    FVector SplineForward = Path->Spline->GetDirectionAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World).GetSafeNormal();
    
    // Calculate right vector based on spline forward and world up
    FVector WorldUp(0, 0, 1);
    FVector SplineRight = FVector::CrossProduct(SplineForward, WorldUp).GetSafeNormal();
    if (SplineRight.IsNearlyZero())
    {
        SplineRight = FVector(0, 1, 0); // Fallback if path is straight up
    }
    
    // Calculate up vector based on forward and right
    FVector SplineUp = FVector::CrossProduct(SplineRight, SplineForward).GetSafeNormal();
    
    // Calculate pitch and roll based on path direction and centripetal force
    float SpeedFactor = FMath::Clamp(CurrentHorizontalSpeed / MaxHorizontalSpeed, 0.0f, 1.0f);
    
    // More pronounced banking in turns (inversely proportional to CurvatureFactor)
    float BankFactor = (1.0f - CurvatureFactor) * 2.0f;
    float TargetRoll = MaxRoll * BankFactor * SpeedFactor;
    
    // Determine bank direction by comparing drone's right vector to curve direction
    FVector CurrentRight = FRotationMatrix(FRotator(0, CurrentRotation.Yaw, 0)).GetUnitAxis(EAxis::Y);
    FVector CurveDirection = (LookAheadPos - CurrentSplinePos).GetSafeNormal();
    float RightDot = FVector::DotProduct(CurveDirection, CurrentRight);
    
    // Apply roll in correct direction based on curve
    TargetRoll *= FMath::Sign(RightDot);
    
    // Calculate pitch based on vertical component of path
    float VerticalFactor = FMath::Clamp(VerticalDelta / LookAheadDistance * 2.0f, -1.0f, 1.0f);
    float TargetPitch = -MaxPitch * VerticalFactor * SpeedFactor;
    
    // Create a target rotation that combines spline yaw with calculated pitch and roll
    FRotator Rotation = SplineRotation;
    Rotation.Pitch = TargetPitch;
    Rotation.Roll = TargetRoll;
    
    // Apply smooth rotation changes (more responsive than before)
    FRotator NewRotation;
    NewRotation.Yaw = FMath::FInterpTo(CurrentRotation.Yaw, Rotation.Yaw,
                                     DeltaTime, YawAcceleration / 45.0f); // More responsive
    NewRotation.Pitch = FMath::FInterpTo(CurrentRotation.Pitch, Rotation.Pitch,
                                       DeltaTime, PitchRollAcceleration * 2.0f); // More responsive
    NewRotation.Roll = FMath::FInterpTo(CurrentRotation.Roll, Rotation.Roll,
                                      DeltaTime, PitchRollAcceleration * 2.0f); // More responsive
    
    // Apply the exact position from the spline and the calculated rotation
    SetActorLocationAndRotation(TargetPosition, NewRotation);
}

void APreciseDrone::UpdatePoseFromDistance()
{
    if (!Path || !Path->Spline)
    {
        return;
    }
    
    // Calculate the current CourseDistance from DistanceTraveled
    float SplineLength = Path->Spline->GetSplineLength();
    
    // Update CourseDistance based on DistanceTraveled
    // This handles looping through the spline multiple times
    float Laps1 = FMath::Floor(DistanceTraveled / SplineLength);
    CourseDistance = DistanceTraveled - Laps1 * SplineLength;
    
    // Get exact position and rotation from the spline
    FVector TargetPosition = Path->Spline->GetLocationAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World);
    FRotator SplineRotation = Path->Spline->GetRotationAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World);
    
    // Calculate the direction at this point (for banking on curves)
    FVector SplineForward = Path->Spline->GetDirectionAtDistanceAlongSpline(
        CourseDistance, ESplineCoordinateSpace::World).GetSafeNormal();
    
    // Look ahead to calculate curvature for banking
    float LookAheadDistance = 100.0f; // Fixed look-ahead for sequencer
    float NextDistance = FMath::Min(CourseDistance + LookAheadDistance, SplineLength);
    if (NextDistance == CourseDistance && SplineLength > 0)
    {
        // Handle looping - look back to the beginning of the spline
        NextDistance = FMath::Min(LookAheadDistance, SplineLength);
    }
    
    // Get look-ahead position for curve calculation
    FVector LookAheadPos = Path->Spline->GetLocationAtDistanceAlongSpline(
        NextDistance, ESplineCoordinateSpace::World);
    FVector NextTangent = Path->Spline->GetTangentAtDistanceAlongSpline(
        NextDistance, ESplineCoordinateSpace::World).GetSafeNormal();
    
    // Calculate curvature factor (how sharp the turn is)
    float TangentDot = FVector::DotProduct(SplineForward, NextTangent);
    float CurvatureFactor = FMath::Clamp(TangentDot, 0.5f, 1.0f); // 1.0 = straight, 0.5 = sharp curve
    
    // Calculate vertical component for pitch
    float VerticalDelta = LookAheadPos.Z - TargetPosition.Z;
    float VerticalFactor = FMath::Clamp(VerticalDelta / LookAheadDistance * 2.0f, -1.0f, 1.0f);
    
    // Calculate right vector based on spline forward and world up
    FVector WorldUp(0, 0, 1);
    FVector SplineRight = FVector::CrossProduct(SplineForward, WorldUp).GetSafeNormal();
    if (SplineRight.IsNearlyZero())
    {
        SplineRight = FVector(0, 1, 0); // Fallback if path is vertical
    }
    
    // Calculate banking based on curvature
    float BankFactor = (1.0f - CurvatureFactor) * 2.0f;
    float TargetRoll = -MaxRoll * BankFactor;
    
    // Determine bank direction based on curve direction
    FVector CurveDirection = (LookAheadPos - TargetPosition).GetSafeNormal();
    float RightDot = FVector::DotProduct(CurveDirection, SplineRight);
    TargetRoll *= FMath::Sign(RightDot);
    
    // Calculate pitch based on vertical component of path
    float TargetPitch = -MaxPitch * VerticalFactor;
    
    // Create final rotation combining spline rotation with calculated pitch and roll
    FRotator NewRotation = SplineRotation;
    NewRotation.Pitch = TargetPitch;
    NewRotation.Roll = TargetRoll;
    
    // Set the actor's location and rotation directly for precise sequencer control
    SetActorLocationAndRotation(TargetPosition, NewRotation);
}

bool APreciseDrone::IfCloseToTarget(FVector Location, FRotator Rotation) const
{
    // Calculate position error
    const FVector CurrentLocation = GetActorLocation();
    const FRotator CurrentRotation = GetActorRotation();
    const FVector PositionError = Location - CurrentLocation;
    
    // Position check
    bool PositionOK = PositionError.SizeSquared() < FMath::Square(PositionThreshold);
    
    // Rotation checks
    bool YawOK = FMath::Abs(FMath::FindDeltaAngleDegrees(
                Rotation.Yaw, CurrentRotation.Yaw)) < RotationThreshold;
    
    bool PitchOK = FMath::Abs(FMath::FindDeltaAngleDegrees(
                 Rotation.Pitch, CurrentRotation.Pitch)) < RotationThreshold;
    
    bool RollOK = FMath::Abs(FMath::FindDeltaAngleDegrees(
               Rotation.Roll, CurrentRotation.Roll)) < RotationThreshold;
    
    return PositionOK && YawOK && PitchOK && RollOK;
}

void APreciseDrone::HandleThrottleInput(const FInputActionValue& Value)
{
    VerticalInput = Value.Get<float>();
}

void APreciseDrone::HandleThrottleReleased(const FInputActionValue& Value)
{
    VerticalInput = 0.0f;
}

void APreciseDrone::HandleMovementInput(const FInputActionValue& Value)
{
    MovementInput = Value.Get<FVector2D>();
}

void APreciseDrone::HandleMovementReleased(const FInputActionValue& Value)
{
    MovementInput = FVector2D::ZeroVector;
}

void APreciseDrone::HandleYawInput(const FInputActionValue& Value)
{
    YawInput = Value.Get<float>();
}

void APreciseDrone::HandleYawReleased(const FInputActionValue& Value)
{
    YawInput = 0.0f;
}

void APreciseDrone::HandleLookInput(const FInputActionValue& Value)
{
    const FVector2D LookAxisVector = Value.Get<FVector2D>();
    
    if (APlayerController* PlayerController = Cast<APlayerController>(Controller))
    {
        // Add yaw input to the camera spring arm
        AddControllerYawInput(LookAxisVector.X);
        // Add pitch input to the camera spring arm
        AddControllerPitchInput(LookAxisVector.Y);
    }
}

void APreciseDrone::MoveToTarget(float DeltaTime)
{
    // Get current state
    FVector CurrentLocation = GetActorLocation();
    FRotator CurrentRotation = GetActorRotation();
    
    // Calculate direction to target
    FVector Direction = TargetLocation - CurrentLocation;
    float VerticalDistance = FMath::Abs(Direction.Z);
    
    // Calculate horizontal direction and distance
    FVector HorizontalDirection = Direction;
    HorizontalDirection.Z = 0;
    float HorizontalDistance = HorizontalDirection.Size();
    float DistanceToTarget = FMath::Sqrt(FMath::Square(HorizontalDistance) + FMath::Square(VerticalDistance));
    
    // Normalize horizontal direction if not zero
    if (HorizontalDistance > 0.001f)
    {
        HorizontalDirection.Normalize();
    }
    
    // Calculate target yaw rotation based on movement direction
    FRotator TargetYawRotation = FRotationMatrix::MakeFromX(HorizontalDirection).Rotator();
    
    // Check if we've already reached the position target
    if (DistanceToTarget <= PositionThreshold)
    {
        // At target position, handle final rotation alignment
        FRotator RotationDelta = TargetRotation - CurrentRotation;
        RotationDelta.Normalize();
        
        // Smoothly interpolate to target rotation
        CurrentRotation.Yaw = FMath::FInterpTo(
            CurrentRotation.Yaw, TargetRotation.Yaw, 
            DeltaTime, YawAcceleration / 90.0f);
        
        CurrentRotation.Pitch = FMath::FInterpTo(
            CurrentRotation.Pitch, TargetRotation.Pitch, 
            DeltaTime, PitchRollAcceleration);
        
        CurrentRotation.Roll = FMath::FInterpTo(
            CurrentRotation.Roll, TargetRotation.Roll, 
            DeltaTime, PitchRollAcceleration);
        
        // Apply the rotation
        SetActorRotation(CurrentRotation);
        
        // Gradually decelerate
        CurrentHorizontalSpeed = FMath::FInterpTo(
            CurrentHorizontalSpeed, 0.0f, 
            DeltaTime, HorizontalAcceleration / 100.0f);
        
        CurrentVerticalSpeed = FMath::FInterpTo(
            CurrentVerticalSpeed, 0.0f, 
            DeltaTime, VerticalAcceleration / 100.0f);
        
        // Check if we're close enough to the target
        if (IfCloseToTarget(TargetLocation, TargetRotation))
        {
            // Reached both position and rotation targets
            bUseTarget = false;
        }
    }
    else
    {
        // Calculate desired speeds based on distance to target
        float DesiredHorizontalSpeed = FMath::Min(
            HorizontalDistance * 2.0f, MaxHorizontalSpeed);
        
        float DesiredVerticalSpeed = FMath::Min(
            VerticalDistance * 2.0f, MaxVerticalSpeed) * 
            FMath::Sign(TargetLocation.Z - CurrentLocation.Z);
        
        // Apply acceleration limits
        CurrentHorizontalSpeed = FMath::FInterpTo(
            CurrentHorizontalSpeed, DesiredHorizontalSpeed, 
            DeltaTime, HorizontalAcceleration / 100.0f);
        
        CurrentVerticalSpeed = FMath::FInterpTo(
            CurrentVerticalSpeed, DesiredVerticalSpeed, 
            DeltaTime, VerticalAcceleration / 100.0f);
        
        // Apply total thrust limitation
        float TotalThrust = FMath::Sqrt(
            FMath::Square(CurrentHorizontalSpeed / MaxHorizontalSpeed) + 
            FMath::Square(CurrentVerticalSpeed / MaxVerticalSpeed));
        
        if (TotalThrust > 1.0f)
        {
            CurrentHorizontalSpeed /= TotalThrust;
            CurrentVerticalSpeed /= TotalThrust;
        }
        
        // Calculate movement vector
        FVector HorizontalMovement = HorizontalDirection * CurrentHorizontalSpeed * DeltaTime;
        FVector VerticalMovement(0, 0, CurrentVerticalSpeed * DeltaTime);
        
        // Apply movement
        CurrentLocation += HorizontalMovement + VerticalMovement;
        
        // Calculate pitch and roll based on movement direction
        FVector ForwardVector = FRotationMatrix(FRotator(0, CurrentRotation.Yaw, 0)).GetUnitAxis(EAxis::X);
        FVector RightVector = FRotationMatrix(FRotator(0, CurrentRotation.Yaw, 0)).GetUnitAxis(EAxis::Y);
        
        float ForwardDot = FVector::DotProduct(HorizontalDirection, ForwardVector);
        float RightDot = FVector::DotProduct(HorizontalDirection, RightVector);
        
        float TargetPitch = -MaxPitch * ForwardDot * (CurrentHorizontalSpeed / MaxHorizontalSpeed);
        float TargetRoll = MaxRoll * RightDot * (CurrentHorizontalSpeed / MaxHorizontalSpeed);
        
        // Apply smooth pitch and roll changes
        CurrentRotation.Pitch = FMath::FInterpTo(
            CurrentRotation.Pitch, TargetPitch, 
            DeltaTime, PitchRollAcceleration);
        
        CurrentRotation.Roll = FMath::FInterpTo(
            CurrentRotation.Roll, TargetRoll, 
            DeltaTime, PitchRollAcceleration);
        
        // Smoothly interpolate yaw towards movement direction
        CurrentRotation.Yaw = FMath::FInterpTo(
            CurrentRotation.Yaw, TargetYawRotation.Yaw, 
            DeltaTime, YawAcceleration / 90.0f);
        
        // Apply the new position and rotation
        SetActorLocationAndRotation(CurrentLocation, CurrentRotation);
    }
}

void APreciseDrone::RotateRotors(float DeltaTime)
{
    const float RotationAmount = RotorRotationSpeed * DeltaTime;
    const FRotator ClockwiseRotation(0.0f, RotationAmount, 0.0f);
    const FRotator CounterClockwiseRotation(0.0f, -RotationAmount, 0.0f);
    
    // Rotors 1 and 3 rotate clockwise, 2 and 4 counterclockwise
    RotorMesh1->AddLocalRotation(ClockwiseRotation);
    RotorMesh3->AddLocalRotation(ClockwiseRotation);
    RotorMesh2->AddLocalRotation(CounterClockwiseRotation);
    RotorMesh4->AddLocalRotation(CounterClockwiseRotation);
}