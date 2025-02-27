#pragma once

#include "CoreMinimal.h"
#include "MeshManager.generated.h"

struct FVCCSimConfig;

UCLASS()
class UFMeshManager : public UObject
{
	GENERATED_BODY()

public:
	UFMeshManager();
	void RConfigure(const FVCCSimConfig& Config);

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VCCSim|MeshManager")
	UMaterialInterface* MeshMaterial;

	// GRPC Call
	int32 AddGlobalMesh();
	bool RemoveGlobalMesh(int32 MeshID);
	
protected:
};
