#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "VCCSimGameInstance.generated.h"

UCLASS()
class VCCSIM_API UVCCSimGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	UVCCSimGameInstance();

	virtual void Init() override;

	// Functions for map management
	UFUNCTION(BlueprintCallable, Category = "VCCSim|Maps")
	void LoadMap(const FString& MapName);

	UFUNCTION(BlueprintCallable, Category = "VCCSim|Maps")
	virtual void ReturnToMainMenu() override;

	// Functions for game state
	UFUNCTION(BlueprintCallable, Category = "VCCSim|GameState")
	void SaveGameState();

	UFUNCTION(BlueprintCallable, Category = "VCCSim|GameState")
	void LoadGameState();

	// Map names
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "VCCSim|Maps")
	FString MainMenuMapName;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "VCCSim|Maps")
	TArray<FString> AvailableMaps;

	// Settings getters/setters
	UFUNCTION(BlueprintCallable, Category = "VCCSim|Settings")
	void SetSimulationSpeed(float Speed);

	UFUNCTION(BlueprintPure, Category = "VCCSim|Settings")
	float GetSimulationSpeed() const { return SimulationSpeed; }

private:
	UPROPERTY()
	float SimulationSpeed;
};