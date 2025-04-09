// #pragma once
//
// #include "CoreMinimal.h"
// #include "Components/PrimitiveComponent.h"
// #include "PointCloudRenderComponent.generated.h"
//
// // Structure to hold point data
// USTRUCT()
// struct FPointData
// {
// 	GENERATED_BODY()
//     
// 	FVector Position;
// 	FColor Color;
// 	float Size;
//
// 	FPointData() : Position(0, 0, 0), Color(FColor::White), Size(1.0f) {}
// 	FPointData(const FVector& InPos, const FColor& InColor, float InSize = 1.0f) 
// 		: Position(InPos), Color(InColor), Size(InSize) {}
// };
//
// UCLASS()
// class VCCSIM_API UPointCloudRenderComponent : public UPrimitiveComponent
// {
// 	GENERATED_BODY()
//
// public:
// 	UPointCloudRenderComponent();
//
// 	// Set visible points
// 	void SetVisiblePoints(const TArray<FVector>& Points);
//     
// 	// Set invisible points 
// 	void SetInvisiblePoints(const TArray<FVector>& Points);
//     
// 	// Clear all points
// 	void ClearPoints();
//
// 	// Material for rendering points
// 	UPROPERTY(EditAnywhere, Category = "Rendering")
// 	UMaterialInterface* PointMaterial;
//     
// 	// Point size for visible points
// 	UPROPERTY(EditAnywhere, Category = "Rendering", meta = (ClampMin = "0.1", ClampMax = "10.0"))
// 	float VisiblePointSize = 5.0f;
//     
// 	// Point size for invisible points
// 	UPROPERTY(EditAnywhere, Category = "Rendering", meta = (ClampMin = "0.1", ClampMax = "10.0"))
// 	float InvisiblePointSize = 3.0f;
//
// 	// UPrimitiveComponent interface
// 	virtual FPrimitiveSceneProxy* CreateSceneProxy() override;
// 	virtual FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
//     
// 	// Internal data
// 	TArray<FPointData> Points;
// 	FBoxSphereBounds Bounds;
// };