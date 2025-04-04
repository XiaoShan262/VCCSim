#pragma once

#include "CoreMinimal.h"
#include "DataType/DataMesh.h"
#include "PathPlanner.generated.h"

UCLASS(BlueprintType, Blueprintable)
class VCCSIM_API UPathPlanner : public UObject
{
	GENERATED_BODY()
	
public:
	static void SemiSphericalPath(
		const TArray<FMeshInfo>& InMesh,
		const float Radius,
		const int32 NumPoses,
		const float& SafeHeight,
		TArray<FVector>& OutPositions,
		TArray<FRotator>& OutRotations
	);
	
	static void VisDebugPath(
		const TArray<FVector>& InPositions,
		const TArray<FRotator>& InRotations,
		const float Duration = 5.0f,
		const FColor& Color = FColor::Red
	);
};