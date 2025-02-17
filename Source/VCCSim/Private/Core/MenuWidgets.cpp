#include "Core/MenuWidgets.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Components/Button.h"
#include "Components/TextBlock.h"

void UMenuWidgets::NativeConstruct()
{
    Super::NativeConstruct();

    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: NativeConstruct called"));

    if (!Map1Button || !Map2Button || !Map3Button || !MapTestButton)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: One or more buttons are null! "
                                    "Map1: %s, Map2: %s, Map3: %s, MapTest: %s"), 
            Map1Button ? TEXT("Valid") : TEXT("Null"),
            Map2Button ? TEXT("Valid") : TEXT("Null"),
            Map3Button ? TEXT("Valid") : TEXT("Null"),
            MapTestButton ? TEXT("Valid") : TEXT("Null"));
        return;
    }

    Map1Button->SetIsEnabled(true);
    Map2Button->SetIsEnabled(true);
    Map3Button->SetIsEnabled(true);
    MapTestButton->SetIsEnabled(true);

    Map1Button->OnClicked.AddDynamic(this, &UMenuWidgets::OnMap1Selected);
    Map2Button->OnClicked.AddDynamic(this, &UMenuWidgets::OnMap2Selected);
    Map3Button->OnClicked.AddDynamic(this, &UMenuWidgets::OnMap3Selected);
    MapTestButton->OnClicked.AddDynamic(this, &UMenuWidgets::OnMapTestSelected);
    
    Map1OriginalColor = Map1Button->GetColorAndOpacity();
    Map2OriginalColor = Map2Button->GetColorAndOpacity();
    Map3OriginalColor = Map3Button->GetColorAndOpacity();
    MapTestOriginalColor = MapTestButton->GetColorAndOpacity();

    Map1Button->OnHovered.AddDynamic(this, &UMenuWidgets::OnMap1Hovered);
    Map1Button->OnUnhovered.AddDynamic(this, &UMenuWidgets::OnMap1Unhovered);
    Map2Button->OnHovered.AddDynamic(this, &UMenuWidgets::OnMap2Hovered);
    Map2Button->OnUnhovered.AddDynamic(this, &UMenuWidgets::OnMap2Unhovered);
    Map3Button->OnHovered.AddDynamic(this, &UMenuWidgets::OnMap3Hovered);
    Map3Button->OnUnhovered.AddDynamic(this, &UMenuWidgets::OnMap3Unhovered);
    MapTestButton->OnHovered.AddDynamic(this, &UMenuWidgets::OnMapTestHovered);
    MapTestButton->OnUnhovered.AddDynamic(this, &UMenuWidgets::OnMapTestUnhovered);

    // Initialize game instance
    GameInstance = Cast<URatSimGameInstance>(GetGameInstance());
    
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: Failed to get RatSimGameInstance"));
        if (StatusText)
        {
            StatusText->SetText(FText::FromString(TEXT("Error: Game instance not found")));
        }
        return;
    }

    // Initialize available maps
    if (GameInstance->AvailableMaps.IsEmpty())
    {
        GameInstance->AvailableMaps.Add(TEXT("Bunker"));
        GameInstance->AvailableMaps.Add(TEXT("Shipping_Port"));
    }

    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Ready - Click a map to begin")));
    }
    
    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: Initialization complete"));
}

void UMenuWidgets::OnMap1Hovered()
{
    if (Map1Button)
    {
        Map1Button->SetColorAndOpacity(HoveredColor);
    }
}

void UMenuWidgets::OnMap2Hovered()
{
    if (Map2Button)
    {
        Map2Button->SetColorAndOpacity(HoveredColor);
    }
}

void UMenuWidgets::OnMap3Hovered()
{
    if (Map3Button)
    {
        Map3Button->SetColorAndOpacity(HoveredColor);
    }
}

void UMenuWidgets::OnMapTestHovered()
{
    if (MapTestButton)
    {
        MapTestButton->SetColorAndOpacity(HoveredColor);
    }
}

void UMenuWidgets::OnMap1Unhovered()
{
    if (Map1Button)
    {
        Map1Button->SetColorAndOpacity(Map1OriginalColor);
    }
}

void UMenuWidgets::OnMap2Unhovered()
{
    if (Map2Button)
    {
        Map2Button->SetColorAndOpacity(Map2OriginalColor);
    }
}

void UMenuWidgets::OnMap3Unhovered()
{
    if (Map3Button)
    {
        Map3Button->SetColorAndOpacity(Map3OriginalColor);
    }
}

void UMenuWidgets::OnMapTestUnhovered()
{
    if (MapTestButton)
    {
        MapTestButton->SetColorAndOpacity(MapTestOriginalColor);
    }
}

void UMenuWidgets::OnMap1Selected()
{
    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: Map1 Button Clicked"));
    
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: GameInstance is null in OnMap1Selected"));
        return;
    }

    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Loading Bunker Map...")));
    }

    // Add a small delay to ensure the loading text is visible
    GetWorld()->GetTimerManager().SetTimer(
        LoadingTimerHandle,
        [this]()
        {
            if (GameInstance)
            {
                GameInstance->LoadMap(GameInstance->AvailableMaps[0]);
            }
        },
        0.1f,
        false
    );
}

void UMenuWidgets::OnMap2Selected()
{
    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: Map2 Button Clicked"));
    
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: GameInstance is null in OnMap2Selected"));
        return;
    }

    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Loading Shipping Port Map...")));
    }

    // Add a small delay to ensure the loading text is visible
    GetWorld()->GetTimerManager().SetTimer(
        LoadingTimerHandle,
        [this]()
        {
            if (GameInstance)
            {
                GameInstance->LoadMap(GameInstance->AvailableMaps[1]);
            }
        },
        0.1f,
        false
    );
}

void UMenuWidgets::OnMap3Selected()
{
    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: Map3 Button Clicked"));
    
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: GameInstance is null in OnMap3Selected"));
        return;
    }

    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Loading Map3...")));
    }

    // Add a small delay to ensure the loading text is visible
    GetWorld()->GetTimerManager().SetTimer(
        LoadingTimerHandle,
        [this]()
        {
            if (GameInstance)
            {
                GameInstance->LoadMap(GameInstance->AvailableMaps[2]);
            }
        },
        0.1f,
        false
    );
}

void UMenuWidgets::OnMapTestSelected()
{
    UE_LOG(LogTemp, Warning, TEXT("MenuWidgets: MapTest Button Clicked"));
    
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MenuWidgets: GameInstance is null in OnMapTestSelected"));
        return;
    }

    if (StatusText)
    {
        StatusText->SetText(FText::FromString(TEXT("Loading Test Map...")));
    }

    // Add a small delay to ensure the loading text is visible
    GetWorld()->GetTimerManager().SetTimer(
        LoadingTimerHandle,
        [this]()
        {
            if (GameInstance)
            {
                GameInstance->LoadMap(GameInstance->AvailableMaps[3]);
            }
        },
        0.1f,
        false
    );
}

/* --------------------------------Pause Menu---------------------------------*/ 

void UPauseMenuWidget::NativeConstruct()
{
    Super::NativeConstruct();

    // Get GameInstance reference
    GameInstance = Cast<URatSimGameInstance>(GetGameInstance());

    if (ResumeButton)
    {
        ResumeButton->OnClicked.AddDynamic(this, &UPauseMenuWidget::OnResumeClicked);
    }
    if (MainMenuButton)
    {
        MainMenuButton->OnClicked.AddDynamic(this, &UPauseMenuWidget::OnMainMenuClicked);
    }
    if (QuitButton)
    {
        QuitButton->OnClicked.AddDynamic(this, &UPauseMenuWidget::OnQuitClicked);
    }
}

void UPauseMenuWidget::OnResumeClicked()
{
    if (APlayerController*PC = GetWorld()->GetFirstPlayerController())
    {
        PC->SetInputMode(FInputModeGameOnly());
        PC->SetShowMouseCursor(false);
    }

    UGameplayStatics::SetGamePaused(GetWorld(), false);
    RemoveFromParent();
}

void UPauseMenuWidget::OnMainMenuClicked()
{
    if (GameInstance)
    {
        // First unpause the game
        UGameplayStatics::SetGamePaused(GetWorld(), false);
        
        // Then return to main menu
        GameInstance->ReturnToMainMenu();
    }
}

void UPauseMenuWidget::OnQuitClicked()
{
    // Save game state before quitting
    if (GameInstance)
    {
        GameInstance->SaveGameState();
    }
    
    UKismetSystemLibrary::QuitGame(GetWorld(), GetWorld()->GetFirstPlayerController(), 
        EQuitPreference::Quit, false);
}