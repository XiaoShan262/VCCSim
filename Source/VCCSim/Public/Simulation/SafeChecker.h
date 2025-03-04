#pragma once
#include "CoreMinimal.h"
#include "SafeChecker.generated.h"

UCLASS()
class AFChecker : public AActor
{
	GENERATED_BODY()

public:

	// AFChecker();
	//
	// // rpc call
	// bool SafeCheck();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float SafeDistance = 500.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float SafeHeight = 500.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float MaxHeight = 10000.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float MaxX = 10000.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float MinX = -10000.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float MaxY = 10000.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SafeChecker")
	float MinY = -10000.0f;

protected:
	virtual void BeginPlay() override;

private:
	// Fst XXX ? in it() ?
			// no exist: struct ? in it ?
			//  1. TArray<primitive shape>
			//  2. octree ? ???
	// Check the closest actor ? ue? space check?

	// 64 LineTraceSingleByChannel? 

	
	// 	const bool bHit = GetWorld()->LineTraceSingleByChannel(
	// 	HitResult,
	// 	CachedStartPoints[GroupIndex],
	// 	CachedEndPoints[GroupIndex],
	// 	ECC_Visibility,
	// 	QueryParams
	// );


	// Make pawnbase as the parent class of carpawn
	// Pawn base beginplay found class component? hud.cpp line 287
	// Path spline -> viewer spline
};
