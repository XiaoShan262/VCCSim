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
#include "GameFramework/Pawn.h"
#include "PawnBase.h"
#include "FlashPawn.generated.h"

class AVCCSimPath;

UCLASS()
class AFlashPawn final: public APawnBase
{
	GENERATED_BODY()
	
public:
	AFlashPawn();
	
	UFUNCTION(BlueprintCallable, Category = "API")
	void SetTarget(FVector Location, FRotator Rotation);
	UFUNCTION(BlueprintCallable, Category = "API")
	void SetPath(const TArray<FVector>& Positions, const TArray<FRotator>& Rotations);
	UFUNCTION(BlueprintCallable, Category = "API")
	bool IsReady() const { return bAcqReady; }
	UFUNCTION(BlueprintCallable, Category = "API")
	void MoveToNext() { bMoveReady = true; bAcqReady = false; }

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VCCSim|Target")
	bool bMoveReady = true;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VCCSim|Target")
	bool bAcqReady = false;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Target")
	AVCCSimPath* Path;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Target")
	bool bUsePath = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VCCSim|Target")
	int32 CurrentPathIndex = 0;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Target")
	FVector TargetLocation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Target")
	FRotator TargetRotation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VCCSim|Target")
	bool bUseTarget = false;
	
protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	UPROPERTY()
	class USceneComponent* FlashRoot;
};
