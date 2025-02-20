#include "Pawns/DronePawn.h"
#include "Components/BoxComponent.h"
#include "Simulation/Recorder.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "InputMappingContext.h"
#include "InputAction.h"

AQuadcopterDrone::AQuadcopterDrone()
{
    PrimaryActorTick.bCanEverTick = true;

    DroneCollision = CreateDefaultSubobject<UBoxComponent>(TEXT("DroneCollision"));
    RootComponent = DroneCollision;
    
    DroneCollision->SetCollisionProfileName(TEXT("PhysicsActor"));
    DroneCollision->SetSimulatePhysics(true);
    DroneCollision->SetAngularDamping(VerticalDamping);
    DroneCollision->SetLinearDamping(HorizontalDamping);
    
    DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DroneMesh"));
    DroneMesh->SetupAttachment(DroneCollision);
    DroneMesh->SetCollisionProfileName(TEXT("NoCollision"));
    
    Rotor1 = CreateDefaultSubobject<USceneComponent>(TEXT("Rotor1"));
    Rotor2 = CreateDefaultSubobject<USceneComponent>(TEXT("Rotor2"));
    Rotor3 = CreateDefaultSubobject<USceneComponent>(TEXT("Rotor3"));
    Rotor4 = CreateDefaultSubobject<USceneComponent>(TEXT("Rotor4"));

    Rotor1->SetupAttachment(DroneCollision);
    Rotor2->SetupAttachment(DroneCollision);
    Rotor3->SetupAttachment(DroneCollision);
    Rotor4->SetupAttachment(DroneCollision);

    RotorMesh1 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh1"));
    RotorMesh2 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh2"));
    RotorMesh3 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh3"));
    RotorMesh4 = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RotorMesh4"));
    
    RotorMesh1->SetupAttachment(Rotor1);
    RotorMesh2->SetupAttachment(Rotor2);
    RotorMesh3->SetupAttachment(Rotor3);
    RotorMesh4->SetupAttachment(Rotor4);

    RotorMesh1->SetCollisionProfileName(TEXT("NoCollision"));
    RotorMesh2->SetCollisionProfileName(TEXT("NoCollision"));
    RotorMesh3->SetCollisionProfileName(TEXT("NoCollision"));
    RotorMesh4->SetCollisionProfileName(TEXT("NoCollision"));
    
    DroneCollision->bHiddenInSceneCapture = true;
    DroneMesh->bHiddenInSceneCapture = true;
    RotorMesh1->bHiddenInSceneCapture = true;
    RotorMesh2->bHiddenInSceneCapture = true;
    RotorMesh3->bHiddenInSceneCapture = true;
    RotorMesh4->bHiddenInSceneCapture = true;
}

void AQuadcopterDrone::BeginPlay()
{
    Super::BeginPlay();

    AddMapContext();
}

void AQuadcopterDrone::SetRecorder(ARecorder* InRecorder)
{
    Recorder = InRecorder;
}

void AQuadcopterDrone::SetRecordInterval(const float& Interval)
{
    RecordInterval = Interval;
    bRecorded = true;
}

void AQuadcopterDrone::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bRecorded)
    {
        TimeSinceLastCapture += DeltaTime;
        if (TimeSinceLastCapture >= RecordInterval)
        {
            TimeSinceLastCapture = 0.0f;
            FPoseData PoseData;
            PoseData.Location = GetActorLocation();
            PoseData.Rotation = GetActorRotation();
            Recorder->SubmitPoseData(this, MoveTemp(PoseData));
        }
    }

    DealWithTarget(DeltaTime);
    ApplyThrust(DeltaTime);
    ApplyRotation(DeltaTime);
    StabilizeDrone(DeltaTime);
    
    const float RotationAmount = RotorRotationSpeed * DeltaTime;
    FRotator ClockwiseRotation(0.0f, RotationAmount, 0.0f);
    FRotator CounterClockwiseRotation(0.0f, -RotationAmount, 0.0f);
    
    // Rotors 1 and 3 rotate clockwise, 2 and 4 counterclockwise
    RotorMesh1->AddLocalRotation(ClockwiseRotation);
    RotorMesh3->AddLocalRotation(ClockwiseRotation);
    RotorMesh2->AddLocalRotation(CounterClockwiseRotation);
    RotorMesh4->AddLocalRotation(CounterClockwiseRotation);
}

void AQuadcopterDrone::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    EnhancedInputComponent = CastChecked<UEnhancedInputComponent>(PlayerInputComponent);

    if (EnhancedInputComponent)
    {
        EnhancedInputComponent->BindAction(ThrottleAction, ETriggerEvent::Triggered,
            this, &AQuadcopterDrone::HandleThrottleInput);
        EnhancedInputComponent->BindAction(ThrottleAction, ETriggerEvent::Completed,
            this, &AQuadcopterDrone::HandleThrottleReleased);
        
        EnhancedInputComponent->BindAction(MovementAction, ETriggerEvent::Triggered,
            this, &AQuadcopterDrone::HandleMovementInput);
        EnhancedInputComponent->BindAction(MovementAction, ETriggerEvent::Completed,
            this, &AQuadcopterDrone::HandleMovementReleased);
        
        EnhancedInputComponent->BindAction(YawAction, ETriggerEvent::Triggered,
            this, &AQuadcopterDrone::HandleYawInput);
        EnhancedInputComponent->BindAction(YawAction, ETriggerEvent::Completed,
            this, &AQuadcopterDrone::HandleYawReleased);

        EnhancedInputComponent->BindAction(LookAction, ETriggerEvent::Triggered,
            this, &AQuadcopterDrone::HandleLookInput);
    }
}

void AQuadcopterDrone::AddMapContext()
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

void AQuadcopterDrone::SetTarget(FVector Location, FRotator Rotation)
{
    TargetLocation = Location;
    TargetRotation = Rotation;
    bUseTarget = true;
}

bool AQuadcopterDrone::IfCloseToTarget(FVector Location, FRotator Rotation) const  
{
    if (FVector::DistSquared(Location, GetActorLocation()) < FMath::Square(PositionThreshold)
        && FMath::Abs(FMath::FindDeltaAngleDegrees(
            Rotation.Yaw, GetActorRotation().Yaw)) < RotationThreshold
        && FMath::Abs(FMath::FindDeltaAngleDegrees(
            Rotation.Pitch, GetActorRotation().Pitch)) < RotationThreshold
        && FMath::Abs(FMath::FindDeltaAngleDegrees(
            Rotation.Roll, GetActorRotation().Roll)) < RotationThreshold)
    {
        return true;
    }
    return false;
}

void AQuadcopterDrone::HandleThrottleInput(const FInputActionValue& Value)
{
    const float TargetValue = Value.Get<float>();
    float DeltaTime = GetWorld()->GetDeltaSeconds();
    float TargetThrottle{TargetValue * MaxThrust};
    if (FMath::Sign(TargetValue) < 0)
    {
        TargetThrottle += 2 * HoverThrust;
    }
    CurrentThrottle = FMath::FInterpTo(CurrentThrottle, TargetThrottle,
        DeltaTime, ThrustIncreaseSpeed);
}

void AQuadcopterDrone::HandleThrottleReleased(const FInputActionValue& Value)
{
    CurrentThrottle = HoverThrust;
}

void AQuadcopterDrone::HandleMovementInput(const FInputActionValue& Value)
{
    const FVector2D MovementVector = Value.Get<FVector2D>();
    
    const FRotator YawRotation(0.0f, GetActorRotation().Yaw, 0.0f);
    const FVector ForwardDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
    const FVector RightDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
    
    const FVector MovementForce =
        (ForwardDirection * MovementVector.Y + RightDirection * MovementVector.X)
        * HorizontalMovementSpeed;
    
    DroneCollision->AddForce(MovementForce);
    
    CurrentPitch = -MovementVector.Y;
    CurrentRoll = MovementVector.X;
}

void AQuadcopterDrone::HandleMovementReleased(const FInputActionValue& Value)
{
    CurrentPitch = 0.0f;
    CurrentRoll = 0.0f;
}

void AQuadcopterDrone::HandleYawInput(const FInputActionValue& Value)
{
    CurrentYaw = Value.Get<float>();
}

void AQuadcopterDrone::HandleYawReleased(const FInputActionValue& Value)
{
    CurrentYaw = 0.0f;
}

void AQuadcopterDrone::HandleLookInput(const FInputActionValue& Value)
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

void AQuadcopterDrone::DealWithTarget(float DeltaTime)
{
    if (!bUseTarget)
    {
        return;
    }

    const FVector CurrentLocation = GetActorLocation();
    const FRotator CurrentRotation = GetActorRotation();
    
    FVector DirectionToTarget = TargetLocation - CurrentLocation;
    
    if (DirectionToTarget.Length() > PositionThreshold)
    {
        DirectionToTarget.Normalize();
        
        const FRotator YawRotation(0.0f, CurrentRotation.Yaw, 0.0f);
        const FVector LocalDirection = YawRotation.UnrotateVector(DirectionToTarget);
        
        const float ForwardMovement = LocalDirection.X;
        const float RightMovement = LocalDirection.Y;
        
        const FVector MovementForce = (GetActorForwardVector() * ForwardMovement + 
                               GetActorRightVector() * RightMovement) * 
                               HorizontalMovementSpeed;
        
        DroneCollision->AddForce(MovementForce);
        
        const float InputRangeScale = FMath::Sqrt(2.0f);
        CurrentPitch = -ForwardMovement * InputRangeScale;
        CurrentRoll = RightMovement * InputRangeScale;
    }
    else
    {
        CurrentPitch = 0.0f;
        CurrentRoll = 0.0f;
    }
    
    const float YawDiff = FMath::FindDeltaAngleDegrees(CurrentRotation.Yaw, TargetRotation.Yaw);
    if (FMath::Abs(YawDiff) > RotationThreshold)
    {
        CurrentYaw = 1.f * FMath::Sign(YawDiff);
    }
    else
    {
        CurrentYaw = 0.0f;
    }
    
    const float HeightDiff = TargetLocation.Z - CurrentLocation.Z;
    float TargetThrottle = FMath::Sign(HeightDiff) * MaxThrust;
    
    if (FMath::Abs(HeightDiff) < PositionThreshold)
    {
        CurrentThrottle = HoverThrust;
    }
    else
    {
        if (HeightDiff < 0)
        {
            TargetThrottle += 2 * HoverThrust;
        }
        CurrentThrottle = FMath::FInterpTo(CurrentThrottle, TargetThrottle,
            DeltaTime, ThrustIncreaseSpeed);
    }
        
    if (IfCloseToTarget(TargetLocation, TargetRotation))
    {
        bUseTarget = false;
    }
}

void AQuadcopterDrone::ApplyThrust(float DeltaTime)
{
    // Calculate base vertical thrust
    FVector VerticalThrust = FVector(0.0f, 0.0f, CurrentThrottle);
    
    DroneCollision->AddForceAtLocation(VerticalThrust / 4, Rotor1->GetComponentLocation());
    DroneCollision->AddForceAtLocation(VerticalThrust / 4, Rotor2->GetComponentLocation());
    DroneCollision->AddForceAtLocation(VerticalThrust / 4, Rotor3->GetComponentLocation());
    DroneCollision->AddForceAtLocation(VerticalThrust / 4, Rotor4->GetComponentLocation());
}

void AQuadcopterDrone::ApplyRotation(float DeltaTime)
{
    const FRotator CurrentRotation = DroneCollision->GetComponentRotation();
    
    const float TargetPitch = CurrentPitch * MaxTiltAngle;
    const float TargetRoll = CurrentRoll * MaxTiltAngle;
    
    const float NewPitch = FMath::FInterpTo(
        CurrentRotation.Pitch, TargetPitch, DeltaTime, TiltSpeed);
    const float NewRoll = FMath::FInterpTo(
        CurrentRotation.Roll, TargetRoll, DeltaTime, TiltSpeed);
    const float NewYaw = CurrentRotation.Yaw + (CurrentYaw * YawSpeed * DeltaTime);

    // Set new rotation on collision component
    // TODO: Use real forces to rotate the drone?
    DroneCollision->SetWorldRotation({NewPitch, NewYaw, NewRoll});
}

void AQuadcopterDrone::StabilizeDrone(float DeltaTime)
{
    const FVector CurrentVelocity = DroneCollision->GetPhysicsLinearVelocity();
    const FVector AngularVelocity = DroneCollision->GetPhysicsAngularVelocityInDegrees();
    
    // Apply stronger damping to vertical movement
    FVector StabilizationForce = FVector(
        -CurrentVelocity.X * StabilizationStrength * 0.5f,
        -CurrentVelocity.Y * StabilizationStrength * 0.5f,
        -CurrentVelocity.Z * StabilizationStrength
    );
    
    DroneCollision->AddForce(StabilizationForce);
    
    // Angular stabilization remains the same
    FVector StabilizationTorque = -AngularVelocity * StabilizationStrength;
    DroneCollision->AddTorqueInDegrees(StabilizationTorque);
}