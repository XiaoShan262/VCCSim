#include "Simulation/MeshManager.h"

UFMeshManager::UFMeshManager()
{
	if (!MeshMaterial)
	{
		MeshMaterial = Cast<UMaterialInterface>(StaticLoadObject(
			UMaterialInterface::StaticClass(), nullptr,
			TEXT("/VCCSim/Materials/M_Dynamic_mesh.M_Dynamic_mesh")));
	}
}
