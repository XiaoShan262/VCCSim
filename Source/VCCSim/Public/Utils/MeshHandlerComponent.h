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
#include "Components/SceneComponent.h"
#include "ProceduralMeshComponent.h"
#include "MeshHandlerComponent.generated.h"

UCLASS(ClassGroup=(VCCSim), meta=(BlueprintSpawnableComponent))
class VCCSIM_API UMeshHandlerComponent : public USceneComponent
{
	GENERATED_BODY()

public:    
	UMeshHandlerComponent();
    
	virtual void OnRegister() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	// Fast mesh update function - called from gRPC thread
	void UpdateMeshFromGRPC(const uint8* MeshData, uint32 DataSize,
		const FTransform& Transform);

	UProceduralMeshComponent* GetMeshComponent() const { return MeshComponent; }

private:
	UPROPERTY()
	UProceduralMeshComponent* MeshComponent = nullptr;

	// Double buffer for mesh data to avoid blocking
	struct FMeshData
	{
		TArray<FVector> Vertices;
		TArray<FVector> Normals;
		TArray<int32> Triangles;
		FTransform Transform;
		bool bNeedsUpdate;
	};
    
	FMeshData MeshBuffers[2];
	int32 CurrentBufferIndex;
	FCriticalSection MeshUpdateLock;

	// Game thread update
	void UpdateMeshInternal();
	void CalculateVertexNormals(const TArray<FVector>& InVertices,
		const TArray<int32>& InTriangles, TArray<FVector>& OutNormals);

	UPROPERTY()
	UMaterialInterface* MeshMaterial;

	// Pre-allocated arrays for mesh creation
	TArray<FVector2D> EmptyUVs;
	TArray<FLinearColor> EmptyColors;
	TArray<FProcMeshTangent> EmptyTangents;

	// Optimization flags
	bool bIsFirstMesh;
	FTimerHandle UpdateTimerHandle;
};