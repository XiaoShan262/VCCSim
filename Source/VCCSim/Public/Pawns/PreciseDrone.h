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

#pragma once

#include "CoreMinimal.h"
#include "DronePawn.h"
#include "GameFramework/Pawn.h"
#include "InputActionValue.h"
#include "PreciseDrone.generated.h"

class ARecorder;

UCLASS()
class VCCSIM_API APreciseDrone : public ADronePawn
{
	GENERATED_BODY()

public:
    APreciseDrone();
    
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(
        class UInputComponent* PlayerInputComponent) override;
    
    virtual bool IfCloseToTarget(FVector Location, FRotator Rotation) const override;
    
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

    void MoveToTarget(float DeltaTime);
    void RotateRotors(float DeltaTime);
};