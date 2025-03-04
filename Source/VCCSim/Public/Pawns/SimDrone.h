#pragma once

#include "CoreMinimal.h"
#include "DronePawn.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include "SimDrone.generated.h"

class ARecorder;

UCLASS()
class VCCSIM_API ASimDrone : public ADronePawn
{
	GENERATED_BODY()

public:
	ASimDrone();
    
	virtual void Tick(float DeltaTime) override;
	
	virtual void SetupPlayerInputComponent(
		class UInputComponent* PlayerInputComponent) override;
    
	virtual bool IfCloseToTarget(FVector Location, FRotator Rotation) const override;
	virtual void FollowThePathAndSteer(float DeltaTime) override;
	// Movement parameters
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Movement")
	float MovementSpeed = 500.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Movement")
	float RotationSpeed = 180.0f;
    
protected:
	virtual void BeginPlay() override;

	UPROPERTY(VisibleAnywhere)
	class UStaticMeshComponent* RotorMesh1;
	UPROPERTY(VisibleAnywhere)
	class UStaticMeshComponent* RotorMesh2;
	UPROPERTY(VisibleAnywhere)
	class UStaticMeshComponent* RotorMesh3;
	UPROPERTY(VisibleAnywhere)
	class UStaticMeshComponent* RotorMesh4;

	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	float RotorRotationSpeed = 3000.0f;

	// Maximum Pitch Angle
	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	float MaxPitchAngle = 25.0f;  
	// Maximum roll angle
	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	float MaxRollAngle = 15.0f;   
	// Tilt Interpolation Speed: It makes the change when you tilt the drone smoother.
	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	float TiltInterpSpeed = 5.0f;
	
	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	float AccelerationSpeed = 5.0f;

	UPROPERTY(EditAnywhere, Category = "VCCSim|Physics")
	bool bEnablePhysical = true;

	
	void HandleThrottleInput(const FInputActionValue& Value);
	void HandleThrottleReleased(const FInputActionValue& Value);
	
	void HandleMovementInput(const FInputActionValue& Value);
	void HandleMovementReleased(const FInputActionValue& Value);
	
	void HandleYawInput(const FInputActionValue& Value);
	void HandleYawReleased(const FInputActionValue& Value);
	
	void HandleLookInput(const FInputActionValue& Value);

private:
	float VerticalInput = 0.0f;
	FVector2D MovementInput = FVector2D::ZeroVector;
	float YawInput = 0.0f;
	
	// float CurrentVerticalSpeed = 0.0f;
	float CurrentMoveSpeed = 0.0f;
	// Maintains the direction of movement before ending the input
	FVector LastMoveDirection = FVector::ZeroVector;
	
	void MoveToTarget(float DeltaTime);
	void RotateRotors(float DeltaTime);
};