#pragma once

#include "CoreMinimal.h"
#include "MeshManager.generated.h"

UCLASS()
class UFMeshManager : public UObject
{
	GENERATED_BODY()

public:
	UFMeshManager();

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VCCSim|MeshManager")
	UMaterialInterface* MeshMaterial;

	// GRPC Call
	int32 AddGlobalMesh();
	bool RemoveGlobalMesh(int32 MeshID);
	
protected:
};

