#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "VCCSimGameMode.generated.h"


UCLASS()
class VCCSIM_API AVCCSimGameMode : public AGameModeBase
{
public:
    GENERATED_BODY()

    virtual void StartPlay() override;

    AVCCSimGameMode(const FObjectInitializer& ObjectInitializer);

private:
	TWeakObjectPtr<UGameUserSettings> UserSettings;
};