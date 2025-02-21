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
