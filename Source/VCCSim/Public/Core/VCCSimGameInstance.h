#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "VCCSimGameInstance.generated.h"

UCLASS()
class VCCSIM_API URatSimGameInstance : public UGameInstance
{
	GENERATED_BODY()

public:
	URatSimGameInstance();

	virtual void Init() override;

	// Functions for map management
	UFUNCTION(BlueprintCallable, Category = "RatSim|Maps")
	void LoadMap(const FString& MapName);

	UFUNCTION(BlueprintCallable, Category = "RatSim|Maps")
	virtual void ReturnToMainMenu() override;

	// Functions for game state
	UFUNCTION(BlueprintCallable, Category = "RatSim|GameState")
	void SaveGameState();

	UFUNCTION(BlueprintCallable, Category = "RatSim|GameState")
	void LoadGameState();

	// Map names
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "RatSim|Maps")
	FString MainMenuMapName;

	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "RatSim|Maps")
	TArray<FString> AvailableMaps;

	// Settings getters/setters
	UFUNCTION(BlueprintCallable, Category = "RatSim|Settings")
	void SetSimulationSpeed(float Speed);

	UFUNCTION(BlueprintPure, Category = "RatSim|Settings")
	float GetSimulationSpeed() const { return SimulationSpeed; }

private:
	UPROPERTY()
	float SimulationSpeed;
};