// Copyright Epic Games, Inc. All Rights Reserved.

#include "Core/VCCSim.h"

#define LOCTEXT_NAMESPACE "FRatSimModule"

void FVCCSimModule::StartupModule()
{
	// This code will execute after your module is loaded into memory;
	// the exact timing is specified in the .uplugin file per-module
	UE_LOG(LogTemp, Warning, TEXT("RatSim module has started!"));
}

void FVCCSimModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.
	// For modules that support dynamic reloading,
	// we call this function before unloading the module.
	UE_LOG(LogTemp, Warning, TEXT("RatSim module has shut down!"));
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FVCCSimModule, RatSim)