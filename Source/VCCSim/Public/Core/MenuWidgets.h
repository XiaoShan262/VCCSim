#pragma once
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Components/Button.h"
#include "Core/VCCSimGameInstance.h"
#include "MenuWidgets.generated.h"


UCLASS()
class VCCSIM_API UMenuWidgets : public UUserWidget
{
	GENERATED_BODY()

protected:
	virtual void NativeConstruct() override;

	UFUNCTION()
	void OnMap1Selected();
    
	UFUNCTION()
	void OnMap2Selected();

	UFUNCTION()
	void OnMap3Selected();

	UFUNCTION()
	void OnMapTestSelected();

	UFUNCTION()
	void OnMap1Hovered();
    
	UFUNCTION()
	void OnMap2Hovered();

	UFUNCTION()
	void OnMap3Hovered();

	UFUNCTION()
	void OnMapTestHovered();

	UFUNCTION()
	void OnMap1Unhovered();
    
	UFUNCTION()
	void OnMap2Unhovered();

	UFUNCTION()
	void OnMap3Unhovered();

	UFUNCTION()
	void OnMapTestUnhovered();

	UPROPERTY()
	FLinearColor Map1OriginalColor;
    
	UPROPERTY()
	FLinearColor Map2OriginalColor;

	UPROPERTY()
	FLinearColor Map3OriginalColor;

	UPROPERTY()
	FLinearColor MapTestOriginalColor;

	UPROPERTY(EditAnywhere, Category = "Button Colors")
	FLinearColor HoveredColor =
		FLinearColor(1.0f, 0.8f, 0.0f, 1.0f);

	UPROPERTY()
	URatSimGameInstance* GameInstance;

	UPROPERTY(meta = (BindWidget))
	class UButton* Map1Button;

	UPROPERTY(meta = (BindWidget))
	class UButton* Map2Button;

	UPROPERTY(meta = (BindWidget))
	class UButton* Map3Button;

	UPROPERTY(meta = (BindWidget))
	class UButton* MapTestButton;

	UPROPERTY(meta = (BindWidget))
	class UTextBlock* StatusText;

private:
	FTimerHandle LoadingTimerHandle;
};

UCLASS()
class VCCSIM_API UPauseMenuWidget : public UUserWidget
{
	GENERATED_BODY()

protected:
	virtual void NativeConstruct() override;

	UPROPERTY(meta = (BindWidget))
	class UButton* ResumeButton;

	UPROPERTY(meta = (BindWidget))
	class UButton* MainMenuButton;

	UPROPERTY(meta = (BindWidget))
	class UButton* QuitButton;

	UFUNCTION()
	void OnResumeClicked();

	UFUNCTION()
	void OnMainMenuClicked();

	UFUNCTION()
	void OnQuitClicked();

private:
	UPROPERTY()
	URatSimGameInstance* GameInstance;
};