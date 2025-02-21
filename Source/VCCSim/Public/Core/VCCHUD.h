// MIT License
// 
// Copyright (c) 2025 Mingyang Wang
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "EnhancedInputSubsystems.h"
#include "Utils/ConfigParser.h"
#include "VCCHUD.generated.h"

class UVCCSIMDisplayWidget;
class UPauseMenuWidget;
class UInputAction;
class UInputMappingContext;
class ARecorder;
struct FRobotGrpcMaps;

UCLASS()
class VCCSIM_API AVCCHUD : public AHUD
{
	GENERATED_BODY()

public:
	void SetupRecorder(const FVCCSimConfig& Config);
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
	TObjectPtr<UInputAction> ToggleRecordingAction;

	UPROPERTY(EditAnywhere, Category = "IA")
	TObjectPtr<UInputMappingContext> DefaultMappingContext;

protected:
	UFUNCTION()
	void OnPauseActionTriggered();
	UFUNCTION()
	void OnToggleRecordingTriggered();
	void SetupEnhancedInput();
	void SetupWidgetsAndLS(const FVCCSimConfig& Config);
	void SetupMainCharacter(const FVCCSimConfig& Config, TArray<AActor*> FoundPawns);
	FRobotGrpcMaps SetupActors(const FVCCSimConfig& Config);
	APawn* CreatePawn(const FVCCSimConfig& Config, const FRobot& Robot);

private:
	UPROPERTY()
	TObjectPtr<UVCCSIMDisplayWidget> WidgetInstance;
	UPROPERTY()
	TObjectPtr<UPauseMenuWidget> CurrentPauseMenu;
	UPROPERTY()
	AActor* Holder = nullptr;
	UPROPERTY()
	ARecorder* Recorder;
};
