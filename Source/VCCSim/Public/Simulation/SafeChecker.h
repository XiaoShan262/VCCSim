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
