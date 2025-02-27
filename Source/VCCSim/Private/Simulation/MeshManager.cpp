#include "Simulation/MeshManager.h"
#include "Utils/ConfigParser.h"

UFMeshManager::UFMeshManager()
{
}

void UFMeshManager::RConfigure(const FVCCSimConfig& Config)
{
	if (Config.VCCSim.MeshMaterial != "")
	{
		MeshMaterial = Cast<UMaterialInterface>(StaticLoadObject(
			UMaterialInterface::StaticClass(), nullptr, *Config.VCCSim.MeshMaterial));
	}
}
