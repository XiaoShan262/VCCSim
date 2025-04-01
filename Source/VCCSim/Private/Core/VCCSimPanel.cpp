#include "Core/VCCSimPanel.h"
#include "PropertyEditorModule.h"
#include "EditorModeManager.h"
#include "Engine/Selection.h"
#include "LevelEditor.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Pawns/FlashPawn.h"
#include "Pawns/SimPath.h"
#include "EngineUtils.h"
#include "ImageUtils.h"
#include "Misc/DateTime.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "AutomationScreenshotOptions.h"
#include "HighResScreenshot.h"

// Define the tab ID
namespace FVCCSimPanelFactory
{
    const FName TabId = FName("VCCSimPanel");
}

SVCCSimPanel::~SVCCSimPanel()
{
    // Clear arrays before destruction
    FlashPawnOptions.Empty();
    TargetObjectOptions.Empty();
    
    // Unregister from selection events
    if (GEditor && GEditor->GetSelectedActors())
    {
        GEditor->GetSelectedActors()->SelectionChangedEvent.RemoveAll(this);
    }
    
    // Clear timer if active
    if (GEditor && bAutoCaptureInProgress)
    {
        GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
        bAutoCaptureInProgress = false;
    }
}

void SVCCSimPanel::Construct(const FArguments& InArgs)
{
    // Get the property editor module
    FPropertyEditorModule& PropertyEditorModule =
        FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
    
    // Register for selection change events
    USelection* Selection = GEditor->GetSelectedActors();
    if (Selection)
    {
        Selection->SelectionChangedEvent.AddSP(
            SharedThis(this), 
            &SVCCSimPanel::OnSelectionChanged
        );
    }
    
    // Refresh actor lists
    RefreshActorLists();
    
    // Create the widget layout
    ChildSlot
    [
        SNew(SVerticalBox)
        
        // Title
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Image Acquisition Panel"))
            .Font(FCoreStyle::GetDefaultFontStyle("Bold", 14))
        ]
        
        // FlashPawn selection
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            CreatePawnSelectPanel()
        ]
        
        // Target object selection
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            CreateTargetSelectPanel()
        ]
        
        // Pose configuration
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            CreatePoseConfigPanel()
        ]
        
        // Capture panel
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5)
        [
            CreateCapturePanel()
        ]
        
    ];
    
    // Initialize with current selection
    RefreshProperties();
}

void SVCCSimPanel::RefreshActorLists()
{
    // Use Reset() instead of Empty() for safer clearing
    FlashPawnOptions.Reset();
    TargetObjectOptions.Reset();
    
    // Find all FlashPawns in the world
    UWorld* World = GEditor->GetEditorWorldContext().World();
    if (World)
    {
        // Get FlashPawns
        for (TActorIterator<AFlashPawn> It(World); It; ++It)
        {
            AFlashPawn* FlashPawn = *It;
            if (FlashPawn)
            {
                // Use MakeShareable to create proper shared pointers
                FlashPawnOptions.Add(MakeWeakObjectPtr(FlashPawn));
            }
        }
        
        // Get all actors for potential targets
        for (TActorIterator<AActor> It(World); It; ++It)
        {
            AActor* Actor = *It;
            // Skip FlashPawns as targets
            if (Actor && !Actor->IsA<AFlashPawn>())
            {
                TargetObjectOptions.Add(MakeWeakObjectPtr(Actor));
            }
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("FlashPawn Options: %d"), FlashPawnOptions.Num());
    
    // Refresh combo boxes if they've been created
    if (FlashPawnComboBox.IsValid())
    {
        FlashPawnComboBox->RefreshOptions();
    }
    
    if (TargetObjectComboBox.IsValid())
    {
        TargetObjectComboBox->RefreshOptions();
    }
}

void SVCCSimPanel::OnSelectionChanged(UObject* Object)
{
    RefreshProperties();
}

void SVCCSimPanel::RefreshProperties()
{
    TArray<UObject*> SelectedActors;
    
    for (FSelectionIterator It(GEditor->GetSelectedActorIterator()); It; ++It)
    {
        AActor* Actor = static_cast<AActor*>(*It);
        if (Actor)
        {
            SelectedActors.Add(Actor);
            
            // If the selected actor is a FlashPawn, update our selection
            AFlashPawn* FlashPawn = Cast<AFlashPawn>(Actor);
            if (FlashPawn)
            {
                SelectedFlashPawn = FlashPawn;
                
                // Update combo box selection
                for (int32 i = 0; i < FlashPawnOptions.Num(); ++i)
                {
                    if (FlashPawnOptions[i].Get() == Actor)
                    {
                        FlashPawnComboBox->SetSelectedItem(FlashPawnOptions[i]);
                        break;
                    }
                }
            }
            // Otherwise, if it's not a FlashPawn, it could be a target
            else if (!Actor->IsA<AFlashPawn>())
            {
                SelectedTargetObject = Actor;
                
                // Update combo box selection
                for (int32 i = 0; i < TargetObjectOptions.Num(); ++i)
                {
                    if (TargetObjectOptions[i].Get() == Actor)
                    {
                        TargetObjectComboBox->SetSelectedItem(TargetObjectOptions[i]);
                        break;
                    }
                }
            }
        }
    }
}

TSharedRef<SWidget> SVCCSimPanel::CreatePawnSelectPanel()
{
    return SNew(SVerticalBox)
    
    // Title for the section
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(STextBlock)
        .Text(FText::FromString("Select Flash Pawn"))
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
    ]
    
    // FlashPawn combo box
    +SVerticalBox::Slot()
    .AutoHeight()
    [
        SAssignNew(FlashPawnComboBox, SComboBox<TWeakObjectPtr<AActor>>)
        .OptionsSource(&FlashPawnOptions)
        .OnGenerateWidget(this, &SVCCSimPanel::GenerateFlashPawnComboItem)
        .OnSelectionChanged(this, &SVCCSimPanel::OnFlashPawnSelectionChanged)
        .Content()
        [
            SNew(STextBlock)
            .Text(this, &SVCCSimPanel::GetFlashPawnComboText)
        ]
    ]
    
    // Refresh button
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 5, 0, 0)
    [
        SNew(SButton)
        .Text(FText::FromString("Refresh Pawn List"))
        .OnClicked_Lambda([this]() {
            RefreshActorLists();
            return FReply::Handled();
        })
    ];
}

TSharedRef<SWidget> SVCCSimPanel::GenerateFlashPawnComboItem(TWeakObjectPtr<AActor> InItem)
{
    if (InItem.IsValid())
    {
        return SNew(STextBlock)
            .Text(FText::FromString(InItem->GetActorLabel()));
    }
    return SNew(STextBlock).Text(FText::FromString("Invalid"));
}

FText SVCCSimPanel::GetFlashPawnComboText() const
{
    if (FlashPawnComboBox.IsValid() && FlashPawnComboBox->GetSelectedItem().IsValid())
    {
        return FText::FromString(FlashPawnComboBox->GetSelectedItem()->GetActorLabel());
    }
    
    return FText::FromString("Select FlashPawn...");
}

void SVCCSimPanel::OnFlashPawnSelectionChanged(
    TWeakObjectPtr<AActor> NewSelection, ESelectInfo::Type SelectInfo)
{
    if (NewSelection.IsValid())
    {
        SelectedFlashPawn = Cast<AFlashPawn>(NewSelection.Get());
    }
    else
    {
        SelectedFlashPawn = nullptr;
    }
}

TSharedRef<SWidget> SVCCSimPanel::CreateTargetSelectPanel()
{
    return SNew(SVerticalBox)
    
    // Title for the section
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(STextBlock)
        .Text(FText::FromString("Select Target Object"))
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
    ]
    
    // Target object combo box
    +SVerticalBox::Slot()
    .AutoHeight()
    [
        SAssignNew(TargetObjectComboBox, SComboBox<TWeakObjectPtr<AActor>>)
        .OptionsSource(&TargetObjectOptions)
        .OnGenerateWidget(this, &SVCCSimPanel::GenerateTargetObjectComboItem)
        .OnSelectionChanged(this, &SVCCSimPanel::OnTargetObjectSelectionChanged)
        .Content()
        [
            SNew(STextBlock)
            .Text(this, &SVCCSimPanel::GetTargetObjectComboText)
        ]
    ]
    
    // Refresh button
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 5, 0, 0)
    [
        SNew(SButton)
        .Text(FText::FromString("Refresh Target List"))
        .OnClicked_Lambda([this]() {
            RefreshActorLists();
            return FReply::Handled();
        })
    ];
}

TSharedRef<SWidget> SVCCSimPanel::GenerateTargetObjectComboItem(TWeakObjectPtr<AActor> InItem)
{
    if (InItem.IsValid())
    {
        return SNew(STextBlock)
            .Text(FText::FromString(InItem->GetActorLabel()));
    }
    return SNew(STextBlock).Text(FText::FromString("Invalid"));
}

FText SVCCSimPanel::GetTargetObjectComboText() const
{
    if (TargetObjectComboBox.IsValid() && TargetObjectComboBox->GetSelectedItem().IsValid())
    {
        return FText::FromString(TargetObjectComboBox->GetSelectedItem()->GetActorLabel());
    }
    
    return FText::FromString("Select Target...");
}

void SVCCSimPanel::OnTargetObjectSelectionChanged(
    TWeakObjectPtr<AActor> NewSelection, ESelectInfo::Type SelectInfo)
{
    if (NewSelection.IsValid())
    {
        SelectedTargetObject = NewSelection.Get();
    }
    else
    {
        SelectedTargetObject = nullptr;
    }
}

TSharedRef<SWidget> SVCCSimPanel::CreatePoseConfigPanel()
{
    return SNew(SVerticalBox)
    
    // Title for the section
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(STextBlock)
        .Text(FText::FromString("Pose Configuration"))
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
    ]
    
    // Number of poses
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(0, 0, 5, 0)
        .VAlign(VAlign_Center)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Number of Poses:"))
            .MinDesiredWidth(120)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        [
            SAssignNew(NumPosesSpinBox, SNumericEntryBox<int32>)
            .Value(NumPoses)
            .MinValue(1)
            .MaxValue(36)
            .Delta(1)
            .AllowSpin(true)
            .OnValueChanged(SNumericEntryBox<int32>::FOnValueChanged::CreateLambda([this](int32 NewValue) {
                NumPoses = NewValue;
            }))
        ]
    ]
    
    // Radius
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(0, 0, 5, 0)
        .VAlign(VAlign_Center)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Radius (cm):"))
            .MinDesiredWidth(120)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        [
            SAssignNew(RadiusSpinBox, SNumericEntryBox<float>)
            .Value(Radius)
            .MinValue(50.0f)
            .MaxValue(1000.0f)
            .Delta(10.0f)
            .AllowSpin(true)
            .OnValueChanged(SNumericEntryBox<float>::FOnValueChanged::CreateLambda([this](float NewValue) {
                Radius = NewValue;
            }))
        ]
    ]
    
    // Height Offset
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(0, 0, 5, 0)
        .VAlign(VAlign_Center)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Height Offset (cm):"))
            .MinDesiredWidth(120)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        [
            SAssignNew(HeightOffsetSpinBox, SNumericEntryBox<float>)
            .Value(HeightOffset)
            .MinValue(-500.0f)
            .MaxValue(500.0f)
            .Delta(10.0f)
            .AllowSpin(true)
            .OnValueChanged(SNumericEntryBox<float>::FOnValueChanged::CreateLambda([this](float NewValue) {
                HeightOffset = NewValue;
            }))
        ]
    ]
    
    // Generate poses button
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 10, 0, 0)
    .HAlign(HAlign_Center)
    [
        SNew(SButton)
        .Text(FText::FromString("Generate Poses"))
        .OnClicked(this, &SVCCSimPanel::OnGeneratePosesClicked)
    ];
}

TSharedRef<SWidget> SVCCSimPanel::CreateCapturePanel()
{
    return SNew(SVerticalBox)
    
    // Title for the section
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(STextBlock)
        .Text(FText::FromString("Image Capture"))
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
    ]
    
    // Single Capture button
    +SVerticalBox::Slot()
    .AutoHeight()
    .HAlign(HAlign_Center)
    [
        SNew(SButton)
        .Text(FText::FromString("Capture Current View"))
        .OnClicked(this, &SVCCSimPanel::OnCaptureImagesClicked)
    ]
    
    // Auto Capture button
    +SVerticalBox::Slot()
    .AutoHeight()
    .HAlign(HAlign_Center)
    .Padding(0, 5, 0, 0)
    [
        SNew(SButton)
        .Text(FText::FromString("Auto-Capture All Poses"))
        .OnClicked_Lambda([this]() {
            StartAutoCapture();
            return FReply::Handled();
        })
    ];
}

FReply SVCCSimPanel::OnGeneratePosesClicked()
{
    GeneratePosesAroundTarget();
    return FReply::Handled();
}

FReply SVCCSimPanel::OnCaptureImagesClicked()
{
    CaptureImageFromCurrentPose();
    return FReply::Handled();
}

void SVCCSimPanel::GeneratePosesAroundTarget()
{
    if (!SelectedFlashPawn.IsValid() || !SelectedTargetObject.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("Must select both a FlashPawn and a target object"));
        return;
    }

    // Get the target object's location
    FVector TargetLocation = SelectedTargetObject->GetActorLocation();
    
    // Create arrays to store the poses
    TArray<FVector> Positions;
    TArray<FRotator> Rotations;
    
    // Generate poses around the target
    for (int32 i = 0; i < NumPoses; ++i)
    {
        // Calculate angle in radians
        float Angle = 2.0f * PI * static_cast<float>(i) / static_cast<float>(NumPoses);
        
        // Calculate position on circle
        FVector Position = TargetLocation + FVector(
            Radius * FMath::Cos(Angle),
            Radius * FMath::Sin(Angle),
            HeightOffset  // Apply height offset
        );
        
        // Calculate rotation to look at target
        FVector Direction = TargetLocation - Position;
        FRotator Rotation = Direction.Rotation();
        
        // Add to arrays
        Positions.Add(Position);
        Rotations.Add(Rotation);
    }
    
    // Set the path on the FlashPawn
    SelectedFlashPawn->SetPath(Positions, Rotations);
    
    // Reset any ongoing auto-capture
    bAutoCaptureInProgress = false;
    GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
    
    UE_LOG(LogTemp, Display, TEXT("Generated %d poses around target"), NumPoses);
}

void SVCCSimPanel::CaptureImageFromCurrentPose()
{
    if (!SelectedFlashPawn.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("No FlashPawn selected"));
        return;
    }
    
    // Create a directory for saving images if it doesn't exist yet
    if (SaveDirectory.IsEmpty())
    {
        SaveDirectory = FPaths::ProjectSavedDir() / TEXT("VCCSimCaptures") / GetTimestampedFilename();
        IFileManager::Get().MakeDirectory(*SaveDirectory, true);
        UE_LOG(LogTemp, Display, TEXT("Saving images to: %s"), *SaveDirectory);
    }
    
    // Check if the FlashPawn is ready to capture
    if (SelectedFlashPawn->IsReady())
    {
        // Get the viewport
        FViewport* Viewport = GEditor->GetActiveViewport();
        if (!Viewport)
        {
            UE_LOG(LogTemp, Error, TEXT("No active viewport"));
            return;
        }
        
        // Setup high-resolution screenshot options
        FHighResScreenshotConfig& HighResScreenshotConfig = GetHighResScreenshotConfig();
        HighResScreenshotConfig.SetHDRCapture(true);
        HighResScreenshotConfig.FilenameOverride = SaveDirectory / FString::Printf(
            TEXT("Pose_%03d.png"), 
            SelectedFlashPawn->CurrentPathIndex == 0 ? 0 : SelectedFlashPawn->CurrentPathIndex - 1
        );
        
        // Capture the high-resolution screenshot
        FScreenshotRequest::RequestScreenshot(HighResScreenshotConfig.FilenameOverride, false, false);
        
        UE_LOG(LogTemp, Display, TEXT("Saved image: %s"), *HighResScreenshotConfig.FilenameOverride);
        
        // Move to next position
        SelectedFlashPawn->MoveToNext();
        
        // If we've finished capturing all poses, reset
        if (!SelectedFlashPawn->bUsePath)
        {
            SaveDirectory.Empty(); // Reset for next capture session
            UE_LOG(LogTemp, Display, TEXT("Completed capture session"));
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("FlashPawn not ready for capture. Wait for it to reach position."));
    }
}

void SVCCSimPanel::StartAutoCapture()
{
    if (!SelectedFlashPawn.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("No FlashPawn selected"));
        return;
    }
    
    // Create a directory for saving images
    SaveDirectory = FPaths::ProjectSavedDir() / TEXT("VCCSimCaptures") / GetTimestampedFilename();
    IFileManager::Get().MakeDirectory(*SaveDirectory, true);
    UE_LOG(LogTemp, Display, TEXT("Auto-capture started. Saving images to: %s"), *SaveDirectory);
    
    // Start the capture process
    bAutoCaptureInProgress = true;
    
    // Set up a timer to check if the FlashPawn is ready for capture
    GEditor->GetTimerManager()->SetTimer(
        AutoCaptureTimerHandle,
        [this]()
        {
            if (!bAutoCaptureInProgress || !SelectedFlashPawn.IsValid())
            {
                // Stop the timer if auto-capture is cancelled or FlashPawn is invalid
                GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
                bAutoCaptureInProgress = false;
                return;
            }
            
            // Check if the FlashPawn is ready to capture
            if (SelectedFlashPawn->IsReady())
            {
                CaptureImageFromCurrentPose();
                
                // If we've finished capturing all poses, stop the auto-capture
                if (!SelectedFlashPawn->bUsePath)
                {
                    UE_LOG(LogTemp, Display, TEXT("Auto-capture completed"));
                    bAutoCaptureInProgress = false;
                    GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
                    SaveDirectory.Empty(); // Reset for next capture session
                }
            }
        },
        0.5f, // Check every 0.5 seconds
        true  // Looping
    );
}

FString SVCCSimPanel::GetTimestampedFilename() const
{
    FDateTime Now = FDateTime::Now();
    return FString::Printf(TEXT("%04d-%02d-%02d_%02d-%02d-%02d"),
        Now.GetYear(), Now.GetMonth(), Now.GetDay(),
        Now.GetHour(), Now.GetMinute(), Now.GetSecond());
}

void FVCCSimPanelFactory::RegisterTabSpawner(FTabManager& TabManager)
{
    TabManager.RegisterTabSpawner(
        TabId, 
        FOnSpawnTab::CreateLambda([](const FSpawnTabArgs& InArgs) -> TSharedRef<SDockTab> {
            return SNew(SDockTab)
                .TabRole(ETabRole::NomadTab)
                .Label(FText::FromString("Image Acquisition"))
                [
                    SNew(SVCCSimPanel)
                ];
        })
    )
    .SetDisplayName(FText::FromString("Image Acquisition"));
}