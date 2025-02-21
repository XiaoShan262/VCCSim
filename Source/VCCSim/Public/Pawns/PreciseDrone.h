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
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Movement")
    float MovementSpeed = 500.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RatSim|Movement")
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

    UPROPERTY(EditAnywhere, Category = "RatSim|Physics")
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