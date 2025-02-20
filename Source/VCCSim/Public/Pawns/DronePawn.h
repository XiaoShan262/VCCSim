#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include "DronePawn.generated.h"

class ARecorder;

UCLASS()
class VCCSIM_API AQuadcopterDrone : public APawn
{
	GENERATED_BODY()

public:
    AQuadcopterDrone();
    UFUNCTION()
    void SetRecorder(ARecorder* InRecorder);
    UFUNCTION()
    void SetRecordInterval(const float& Interval);
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(
        class UInputComponent* PlayerInputComponent) override;

    UFUNCTION()
    void AddMapContext();
    UFUNCTION(BlueprintCallable, Category = "API")
    void SetTarget(FVector Location, FRotator Rotation);
    UFUNCTION(BlueprintCallable, Category = "API")
    bool IfCloseToTarget(FVector Location, FRotator Rotation) const;

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

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "RatSim|Debug")
    bool bRecorded = false;
    
protected:
    virtual void BeginPlay() override;

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
    class UStaticMeshComponent* DroneMesh;

    UPROPERTY(VisibleAnywhere)
    class UStaticMeshComponent* RotorMesh1;
    UPROPERTY(VisibleAnywhere)
    class UStaticMeshComponent* RotorMesh2;
    UPROPERTY(VisibleAnywhere)
    class UStaticMeshComponent* RotorMesh3;
    UPROPERTY(VisibleAnywhere)
    class UStaticMeshComponent* RotorMesh4;

    UPROPERTY(VisibleAnywhere)
    class USceneComponent* Rotor1;
    UPROPERTY(VisibleAnywhere)
    class USceneComponent* Rotor2;
    UPROPERTY(VisibleAnywhere)
    class USceneComponent* Rotor3;
    UPROPERTY(VisibleAnywhere)
    class USceneComponent* Rotor4;

    UPROPERTY(VisibleAnywhere)
    class UEnhancedInputComponent* EnhancedInputComponent;

    // Physics Parameters
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float HoverThrust = 980.0f;
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float MaxThrust = 2940.0f;
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float ThrustIncreaseSpeed = 50.f;
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float VerticalDamping = 2.0f;

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float MaxTiltAngle = 25.0f;
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float TiltSpeed = 5.f;

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float YawSpeed = 60.f;

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float StabilizationStrength = 5.0f;

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float HorizontalMovementSpeed = 980.0f;
    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float HorizontalDamping = 2.0f;

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
    float RotorRotationSpeed = 3000.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    class UBoxComponent* DroneCollision;

    void HandleThrottleInput(const FInputActionValue& Value);
    void HandleThrottleReleased(const FInputActionValue& Value);
    void HandleMovementInput(const FInputActionValue& Value);
    void HandleMovementReleased(const FInputActionValue& Value);
    void HandleYawInput(const FInputActionValue& Value);
    void HandleYawReleased(const FInputActionValue& Value);
    void HandleLookInput(const FInputActionValue& Value);

private:
    float CurrentThrottle = 0.0f;
    float CurrentPitch = 0.0f;
    float CurrentRoll = 0.0f;
    float CurrentYaw = 0.0f;

    void DealWithTarget(float DeltaTime);
    void ApplyThrust(float DeltaTime);
    void ApplyRotation(float DeltaTime);
    void StabilizeDrone(float DeltaTime);

    UPROPERTY()
    ARecorder* Recorder;
    float RecordInterval = -1.f;
    float TimeSinceLastCapture = 0.f;
};
