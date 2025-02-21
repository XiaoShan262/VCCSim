#include "Pawns/DronePawn.h"
#include "EnhancedInputSubsystems.h"

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