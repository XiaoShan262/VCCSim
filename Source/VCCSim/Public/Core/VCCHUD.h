#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "EnhancedInputSubsystems.h"
#include "Utils/ConfigParser.h"
#include "VCCHUD.generated.h"

class URatSIMDisplayWidget;
class UPauseMenuWidget;
class UInputAction;
class UInputMappingContext;
struct FRobotGrpcMaps;

UCLASS()
class VCCSIM_API ARatHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    
	// Original properties
	UPROPERTY(EditDefaultsOnly, Category = "UI")
	TSubclassOf<UUserWidget> WidgetClass;
    
	UPROPERTY(EditDefaultsOnly, Category = "UI")
	TSubclassOf<UPauseMenuWidget> PauseWidgetClass;
    
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Gameplay")
	TObjectPtr<APawn> MainCharacter;
    
	UPROPERTY(EditAnywhere, Category = "IA")
	TObjectPtr<UInputAction> PauseAction;

	UPROPERTY(EditAnywhere, Category = "IA")
	TObjectPtr<UInputMappingContext> DefaultMappingContext;

protected:
	UFUNCTION()
	void OnPauseActionTriggered();
	void SetupEnhancedInput();
	void SetupWidgetsAndLS(const FVCCSimConfig& Config);
	void SetupMainCharacter(const FVCCSimConfig& Config, TArray<AActor*> FoundPawns);
	FRobotGrpcMaps SetupActors(const FVCCSimConfig& Config);
	APawn* CreatePawn(const FVCCSimConfig& Config, const FRobot& Robot);

private:
	UPROPERTY()
	TObjectPtr<URatSIMDisplayWidget> WidgetInstance;
	UPROPERTY()
	TObjectPtr<UPauseMenuWidget> CurrentPauseMenu;
	UPROPERTY()
	AActor* Holder = nullptr;
};
