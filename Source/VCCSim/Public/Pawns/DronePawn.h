#pragma once

#include "CoreMinimal.h"
#include "Pawns/PawnBase.h"
#include "InputActionValue.h"
#include "DronePawn.generated.h"

class ARecorder;

UCLASS()
class VCCSIM_API ADronePawn : public APawnBase
{
	GENERATED_BODY()

public:

    UFUNCTION()
    void AddMapContext();
    
    UFUNCTION(BlueprintCallable, Category = "API")
    void SetTarget(FVector Location, FRotator Rotation);
    virtual bool IfCloseToTarget(FVector Location, FRotator Rotation) const;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Target")
    FVector TargetLocation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Target")
    FRotator TargetRotation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Target")
    bool bUseTarget = false;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Target")
    float PositionThreshold = 0.5f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Target")
    float RotationThreshold = 0.5f;

    // Enhanced Input Components
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "RatSim|Input")
    class UInputMappingContext* DroneInputMappingContext;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "RatSim|Input")
    class UInputAction* ThrottleAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "RatSim|Input")
    class UInputAction* MovementAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "RatSim|Input")
    class UInputAction* YawAction;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "RatSim|Input")
    class UInputAction* LookAction;
    
    // Components
    UPROPERTY(VisibleAnywhere)
    class USceneComponent* DroneRoot;
    
    UPROPERTY(VisibleAnywhere)
    class UStaticMeshComponent* DroneMesh;

    UPROPERTY(VisibleAnywhere)
    class UEnhancedInputComponent* EnhancedInputComponent;
};
