#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "VCCSimGameMode.generated.h"


UCLASS()
class VCCSIM_API ARatSimGameMode : public AGameModeBase
{
public:
    GENERATED_BODY()

    virtual void StartPlay() override;

    ARatSimGameMode(const FObjectInitializer& ObjectInitializer);

private:
	TWeakObjectPtr<UGameUserSettings> UserSettings;
};