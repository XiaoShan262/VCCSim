#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "LoadObj.generated.h"

USTRUCT()
struct FMesh
{
	GENERATED_BODY()
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<int32> Triangles;
};

class UProceduralMeshComponent;

UCLASS(Blueprintable)
class VCCSIM_API AObjLoader: public AActor
{
	GENERATED_BODY()
public:
	AObjLoader();
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere)
	UMaterialInterface* Material = nullptr;
	
	void LoadObjFiles();
	
	UFUNCTION(BlueprintCallable)
	void UpdateMesh();

// private: for debug
	UPROPERTY()
	UProceduralMeshComponent* MeshComponent = nullptr;
	UPROPERTY()
	TArray<FMesh> Meshes;
};
