#include "Core/VCCSimPanel.h"
#include "PropertyEditorModule.h"
#include "Engine/Selection.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Pawns/FlashPawn.h"
#include "ImageUtils.h"
#include "Misc/DateTime.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Sensors/CameraSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/SegmentCamera.h"
#include "Utils/ImageProcesser.h"

// Destructor
SVCCSimPanel::~SVCCSimPanel()
{
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

// Construct method
void SVCCSimPanel::Construct(const FArguments& InArgs)
{
    // Register for selection change events
    USelection* Selection = GEditor->GetSelectedActors();
    if (Selection)
    {
        Selection->SelectionChangedEvent.AddSP(
            SharedThis(this), 
            &SVCCSimPanel::OnSelectionChanged
        );
    }
    
    // Load logo images
    FString PluginDir = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("VCCSim"));
    FString VCCLogoPath = FPaths::Combine(PluginDir, TEXT("image/Logo/vcc.png"));
    FString SZULogoPath = FPaths::Combine(PluginDir, TEXT("image/Logo/szu.png"));
    
    // Create dynamic brushes if files exist
    if (FPaths::FileExists(VCCLogoPath))
    {
        VCCLogoBrush = MakeShareable(new FSlateDynamicImageBrush(
            FName(*VCCLogoPath), 
            FVector2D(65, 65),  // Maintain square ratio but smaller for UI
            FLinearColor(1, 1, 1, 1)));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("VCC logo file not found at: %s"), *VCCLogoPath);
    }

    if (FPaths::FileExists(SZULogoPath))
    {
        // Calculate width to maintain aspect ratio with same height as VCC logo
        float SZUWidth = 80 * (272.0f / 80.0f);  // 85 * 3.4 = ~289
        SZULogoBrush = MakeShareable(new FSlateDynamicImageBrush(
            FName(*SZULogoPath), 
            FVector2D(SZUWidth, 80),  // Same height as VCC logo, width preserves ratio
            FLinearColor(1, 1, 1, 1)));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("SZU logo file not found at: %s"), *SZULogoPath);
    }
    
    // Create the widget layout
    ChildSlot
    [
        SNew(SVerticalBox)
        
        // Logo panel at the top
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(8)
        [
            SNew(SHorizontalBox)
            
            // VCC Logo (left)
            +SHorizontalBox::Slot()
            .AutoWidth()
            .Padding(8, 0, 0, 0)
            .HAlign(HAlign_Left)
            .VAlign(VAlign_Center)
            [
                SNew(SImage)
                .Image_Lambda([this]() {
                    return VCCLogoBrush.IsValid() ? VCCLogoBrush.Get() : FCoreStyle::Get().GetBrush("NoBrush");
                })
            ]
            
            // Spacer to push logos to the edges
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            [
                SNew(SSpacer)
            ]
            
            // SZU Logo (right)
            +SHorizontalBox::Slot()
            .AutoWidth()
            .HAlign(HAlign_Right)
            .VAlign(VAlign_Center)
            [
                SNew(SImage)
                .Image_Lambda([this]() {
                    return SZULogoBrush.IsValid() ? SZULogoBrush.Get() : FCoreStyle::Get().GetBrush("NoBrush");
                })
            ]
        ]
        
        // Section separator after logos
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5, 0)
        [
            SNew(SSeparator)
            .Thickness(2.0f)
            .ColorAndOpacity(FLinearColor(0.3f, 0.3f, 0.3f, 1.0f))
        ]
        
        // FlashPawn selection
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(10)
        [
            CreatePawnSelectPanel()
        ]
        
        // Section separator
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5, 0)
        [
            SNew(SSeparator)
            .Thickness(2.0f)
            .ColorAndOpacity(FLinearColor(0.3f, 0.3f, 0.3f, 1.0f))
        ]

        // Camera selection
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(10)
        [
            CreateCameraSelectPanel()
        ]
        
        // Section separator
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5, 0)
        [
            SNew(SSeparator)
            .Thickness(2.0f)
            .ColorAndOpacity(FLinearColor(0.3f, 0.3f, 0.3f, 1.0f))
        ]
                
        // Target object selection
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(10)
        [
            CreateTargetSelectPanel()
        ]
        
        // Section separator
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5, 0)
        [
            SNew(SSeparator)
            .Thickness(2.0f)
            .ColorAndOpacity(FLinearColor(0.3f, 0.3f, 0.3f, 1.0f))
        ]
        
        // Pose configuration
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(10)
        [
            CreatePoseConfigPanel()
        ]
        
        // Section separator
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(5, 0)
        [
            SNew(SSeparator)
            .Thickness(2.0f)
            .ColorAndOpacity(FLinearColor(0.3f, 0.3f, 0.3f, 1.0f))
        ]
        
        // Capture panel
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(10)
        [
            CreateCapturePanel()
        ]
    ];
}

// Selection panel for FlashPawn
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
    
    // Selected FlashPawn display
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
            .Text(FText::FromString("Current:"))
            .MinDesiredWidth(40)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
            .Padding(3)
            [
                SAssignNew(SelectedFlashPawnText, STextBlock)
                .Text(FText::FromString("None selected"))
            ]
        ]
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 3)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Toggle selection mode
    +SVerticalBox::Slot()
    .AutoHeight()
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(0, 0, 5, 0)
        .VAlign(VAlign_Center)
        [
            SAssignNew(SelectFlashPawnToggle, SCheckBox)
            .IsChecked(bSelectingFlashPawn ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
            .OnCheckStateChanged(this, &SVCCSimPanel::OnSelectFlashPawnToggleChanged)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        .VAlign(VAlign_Center)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Click to select FlashPawn"))
        ]
    ];
}

// Selection panel for Target Object
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
    
    // Selected Target display
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
            .Text(FText::FromString("Current:"))
            .MinDesiredWidth(40)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
            .Padding(3)
            [
                SAssignNew(SelectedTargetObjectText, STextBlock)
                .Text(FText::FromString("None selected"))
            ]
        ]
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 3)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Toggle selection mode
    +SVerticalBox::Slot()
    .AutoHeight()
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(0, 0, 5, 0)
        .VAlign(VAlign_Center)
        [
            SAssignNew(SelectTargetToggle, SCheckBox)
            .IsChecked(bSelectingTarget ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
            .OnCheckStateChanged(this, &SVCCSimPanel::OnSelectTargetToggleChanged)
        ]
        +SHorizontalBox::Slot()
        .FillWidth(1.0f)
        .VAlign(VAlign_Center)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Click to select Target"))
        ]
    ];
}

// Camera selection panel
TSharedRef<SWidget> SVCCSimPanel::CreateCameraSelectPanel()
{
    return SNew(SVerticalBox)
    
    // Title for the section
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(STextBlock)
        .Text(FText::FromString("Camera Selection"))
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 12))
    ]
    
    // Camera Availability Row (displays what cameras are available)
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(SVerticalBox)
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(0, 0, 0, 2)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Available Cameras:"))
            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 10))
        ]
        +SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SHorizontalBox)
            
            // RGB Camera availability
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SNew(SImage)
                        .Image_Lambda([this]() {
                            return bHasRGBCamera ? 
                                FCoreStyle::Get().GetBrush("Icons.Checkmark") : 
                                FCoreStyle::Get().GetBrush("Icons.X");
                        })
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasRGBCamera ? 
                                FLinearColor(0.0f, 0.8f, 0.0f, 1.0f) : 
                                FLinearColor(0.8f, 0.0f, 0.0f, 1.0f);
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("RGB"))
                    ]
                ]
            ]
            
            // Depth Camera availability
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SNew(SImage)
                        .Image_Lambda([this]() {
                            return bHasDepthCamera ? 
                                FCoreStyle::Get().GetBrush("Icons.Checkmark") : 
                                FCoreStyle::Get().GetBrush("Icons.X");
                        })
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasDepthCamera ? 
                                FLinearColor(0.0f, 0.8f, 0.0f, 1.0f) : 
                                FLinearColor(0.8f, 0.0f, 0.0f, 1.0f);
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Depth"))
                    ]
                ]
            ]
            
            // Segmentation Camera availability
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SNew(SImage)
                        .Image_Lambda([this]() {
                            return bHasSegmentationCamera ? 
                                FCoreStyle::Get().GetBrush("Icons.Checkmark") : 
                                FCoreStyle::Get().GetBrush("Icons.X");
                        })
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasSegmentationCamera ? 
                                FLinearColor(0.0f, 0.8f, 0.0f, 1.0f) : 
                                FLinearColor(0.8f, 0.0f, 0.0f, 1.0f);
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Segmentation"))
                    ]
                ]
            ]
        ]
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 5)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Active Camera Selection Row (controls which cameras to use)
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 5)
    [
        SNew(SVerticalBox)
        +SVerticalBox::Slot()
        .AutoHeight()
        .Padding(0, 0, 0, 2)
        [
            SNew(STextBlock)
            .Text(FText::FromString("Active Cameras:"))
            .Font(FCoreStyle::GetDefaultFontStyle("Regular", 10))
        ]
        +SVerticalBox::Slot()
        .AutoHeight()
        [
            SNew(SHorizontalBox)
            
            // RGB Camera activation
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SAssignNew(RGBCameraCheckBox, SCheckBox)
                        .IsChecked(bUseRGBCamera ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
                        .OnCheckStateChanged(this, &SVCCSimPanel::OnRGBCameraCheckboxChanged)
                        .IsEnabled_Lambda([this]() {
                            return bHasRGBCamera;
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("RGB"))
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasRGBCamera ? FLinearColor::White : FLinearColor::Gray;
                        })
                    ]
                ]
            ]
            
            // Depth Camera activation
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SAssignNew(DepthCameraCheckBox, SCheckBox)
                        .IsChecked(bUseDepthCamera ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
                        .OnCheckStateChanged(this, &SVCCSimPanel::OnDepthCameraCheckboxChanged)
                        .IsEnabled_Lambda([this]() {
                            return bHasDepthCamera;
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Depth"))
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasDepthCamera ? FLinearColor::White : FLinearColor::Gray;
                        })
                    ]
                ]
            ]
            
            // Segmentation Camera activation
            +SHorizontalBox::Slot()
            .FillWidth(1.0f)
            .Padding(5, 0)
            [
                SNew(SBorder)
                .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
                .Padding(3)
                .HAlign(HAlign_Center)
                [
                    SNew(SHorizontalBox)
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    .Padding(0, 0, 5, 0)
                    [
                        SAssignNew(SegmentationCameraCheckBox, SCheckBox)
                        .IsChecked(bUseSegmentationCamera ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
                        .OnCheckStateChanged(this, &SVCCSimPanel::OnSegmentationCameraCheckboxChanged)
                        .IsEnabled_Lambda([this]() {
                            return bHasSegmentationCamera;
                        })
                    ]
                    +SHorizontalBox::Slot()
                    .AutoWidth()
                    .VAlign(VAlign_Center)
                    [
                        SNew(STextBlock)
                        .Text(FText::FromString("Segmentation"))
                        .ColorAndOpacity_Lambda([this]() {
                            return bHasSegmentationCamera ? FLinearColor::White : FLinearColor::Gray;
                        })
                    ]
                ]
            ]
        ]
    ]
    
    // Update button
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 5, 0, 0)
    [
        SNew(SButton)
        .Text(FText::FromString("Update Cameras"))
        .OnClicked_Lambda([this]() {
            UpdateActiveCameras();
            return FReply::Handled();
        })
        .IsEnabled_Lambda([this]() {
            return SelectedFlashPawn.IsValid() && (bHasRGBCamera || bHasDepthCamera || bHasSegmentationCamera);
        })
    ];
}

// Pose configuration panel 
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
    .Padding(0, 0, 0, 2)
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
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
            .Padding(3)
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
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 3)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Radius
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 2)
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
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
            .Padding(3)
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
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 3)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Height Offset
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 0, 0, 2)
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
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("ToolPanel.GroupBorder"))
            .Padding(3)
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
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 3)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Action buttons
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 2, 0, 0)
    [
        SNew(SHorizontalBox)
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(5, 0)
        .HAlign(HAlign_Center)
        [
            SNew(SButton)
            .Text(FText::FromString("Move Back"))
            .OnClicked_Lambda([this]() {
                if (SelectedFlashPawn.IsValid())
                {
                    SelectedFlashPawn->MoveBackward();
                }
                return FReply::Handled();
            })
            .IsEnabled_Lambda([this]() {
                return SelectedFlashPawn.IsValid() && SelectedTargetObject.IsValid();
            })
        ]
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(5, 0)
        .HAlign(HAlign_Center)
        [
            SNew(SButton)
            .Text(FText::FromString("Generate Poses"))
            .OnClicked(this, &SVCCSimPanel::OnGeneratePosesClicked)
            .IsEnabled_Lambda([this]() {
                return SelectedFlashPawn.IsValid() && SelectedTargetObject.IsValid();
            })
        ]
        +SHorizontalBox::Slot()
        .AutoWidth()
        .Padding(5, 0)
        .HAlign(HAlign_Center)
        [
            SNew(SButton)
            .Text(FText::FromString("Move Next"))
            .OnClicked_Lambda([this]() {
                if (SelectedFlashPawn.IsValid())
                {
                    SelectedFlashPawn->MoveForward();
                }
                return FReply::Handled();
            })
            .IsEnabled_Lambda([this]() {
                return SelectedFlashPawn.IsValid() && SelectedTargetObject.IsValid();
            })
        ]
    ];
}

// Capture panel
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
        .ContentPadding(FMargin(10, 5))
        .ButtonColorAndOpacity(FLinearColor(0.2f, 0.4f, 0.6f, 1.0f))
        .Text(FText::FromString("Capture Current View"))
        .OnClicked(this, &SVCCSimPanel::OnCaptureImagesClicked)
        .IsEnabled_Lambda([this]() {
            return SelectedFlashPawn.IsValid() && 
                   (bUseRGBCamera && bHasRGBCamera) || 
                   (bUseDepthCamera && bHasDepthCamera) || 
                   (bUseSegmentationCamera && bHasSegmentationCamera);
        })
    ]
    
    // Row separator
    +SVerticalBox::Slot()
    .AutoHeight()
    .Padding(0, 5)
    [
        SNew(SSeparator)
        .Thickness(1.0f)
        .ColorAndOpacity(FLinearColor(0.5f, 0.5f, 0.5f, 0.5f))
    ]
    
    // Auto Capture button
    +SVerticalBox::Slot()
    .AutoHeight()
    .HAlign(HAlign_Center)
    .Padding(0, 5, 0, 0)
    [
        SNew(SButton)
        .ContentPadding(FMargin(10, 5))
        .ButtonColorAndOpacity(FLinearColor(0.6f, 0.3f, 0.2f, 1.0f))
        .Text(FText::FromString("Auto-Capture All Poses"))
        .OnClicked_Lambda([this]() {
            StartAutoCapture();
            return FReply::Handled();
        })
        .IsEnabled_Lambda([this]() {
            return SelectedFlashPawn.IsValid() && 
                   ((bUseRGBCamera && bHasRGBCamera) || 
                    (bUseDepthCamera && bHasDepthCamera) || 
                    (bUseSegmentationCamera && bHasSegmentationCamera));
        })
    ];
}

// Selection state toggle callbacks
void SVCCSimPanel::OnSelectFlashPawnToggleChanged(ECheckBoxState NewState)
{
    bSelectingFlashPawn = (NewState == ECheckBoxState::Checked);
    
    // If turning on FlashPawn selection, disable target selection
    if (bSelectingFlashPawn && bSelectingTarget)
    {
        bSelectingTarget = false;
        SelectTargetToggle->SetIsChecked(ECheckBoxState::Unchecked);
    }
}

void SVCCSimPanel::OnSelectTargetToggleChanged(ECheckBoxState NewState)
{
    bSelectingTarget = (NewState == ECheckBoxState::Checked);
    
    // If turning on Target selection, disable FlashPawn selection
    if (bSelectingTarget && bSelectingFlashPawn)
    {
        bSelectingFlashPawn = false;
        SelectFlashPawnToggle->SetIsChecked(ECheckBoxState::Unchecked);
    }
}

// Camera checkbox callbacks
void SVCCSimPanel::OnRGBCameraCheckboxChanged(ECheckBoxState NewState)
{
    bUseRGBCamera = (NewState == ECheckBoxState::Checked);
}

void SVCCSimPanel::OnDepthCameraCheckboxChanged(ECheckBoxState NewState)
{
    bUseDepthCamera = (NewState == ECheckBoxState::Checked);
}

void SVCCSimPanel::OnSegmentationCameraCheckboxChanged(ECheckBoxState NewState)
{
    bUseSegmentationCamera = (NewState == ECheckBoxState::Checked);
}

// Selection changed callback
void SVCCSimPanel::OnSelectionChanged(UObject* Object)
{
    // Skip if we're not in selection mode
    if (!bSelectingFlashPawn && !bSelectingTarget)
    {
        return;
    }
    
    USelection* Selection = GEditor->GetSelectedActors();
    if (!Selection || Selection->Num() == 0)
    {
        return;
    }
    
    // Process only the first selected actor
    AActor* Actor = Cast<AActor>(Selection->GetSelectedObject(0));
    if (!Actor)
    {
        return;
    }
    
    // If we're selecting a FlashPawn
    if (bSelectingFlashPawn)
    {
        AFlashPawn* FlashPawn = Cast<AFlashPawn>(Actor);
        if (FlashPawn)
        {
            SelectedFlashPawn = FlashPawn;
            SelectedFlashPawnText->SetText(FText::FromString(FlashPawn->GetActorLabel()));
            
            // Turn off selection mode
            bSelectingFlashPawn = false;
            SelectFlashPawnToggle->SetIsChecked(ECheckBoxState::Unchecked);
            
            // Check what camera components are available
            CheckCameraComponents();
            
            UE_LOG(LogTemp, Display, TEXT("Selected FlashPawn: %s"), *FlashPawn->GetActorLabel());
        }
    }
    // If we're selecting a target
    else if (bSelectingTarget)
    {
        // Skip if it's a FlashPawn (can't target itself)
        if (!Actor->IsA<AFlashPawn>())
        {
            SelectedTargetObject = Actor;
            SelectedTargetObjectText->SetText(FText::FromString(Actor->GetActorLabel()));
            
            // Turn off selection mode
            bSelectingTarget = false;
            SelectTargetToggle->SetIsChecked(ECheckBoxState::Unchecked);
            
            UE_LOG(LogTemp, Display, TEXT("Selected Target: %s"), *Actor->GetActorLabel());
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Cannot select a FlashPawn as a target"));
        }
    }
}

// Check what camera components are available on the selected FlashPawn
void SVCCSimPanel::CheckCameraComponents()
{
    bHasRGBCamera = false;
    bHasDepthCamera = false;
    bHasSegmentationCamera = false;
    
    if (!SelectedFlashPawn.IsValid())
    {
        return;
    }
    
    // Check for RGB cameras
    TArray<URGBCameraComponent*> RGBCameras;
    SelectedFlashPawn->GetComponents<URGBCameraComponent>(RGBCameras);
    bHasRGBCamera = (RGBCameras.Num() > 0);
    
    // Check for Depth cameras
    TArray<UDepthCameraComponent*> DepthCameras;
    SelectedFlashPawn->GetComponents<UDepthCameraComponent>(DepthCameras);
    bHasDepthCamera = (DepthCameras.Num() > 0);
    
    // Check for Segmentation cameras
    TArray<USegmentationCameraComponent*> SegmentationCameras;
    SelectedFlashPawn->GetComponents<USegmentationCameraComponent>(SegmentationCameras);
    bHasSegmentationCamera = (SegmentationCameras.Num() > 0);
    
    UE_LOG(LogTemp, Display, TEXT("FlashPawn camera components: RGB=%d, Depth=%d, Segmentation=%d"),
        bHasRGBCamera ? 1 : 0, bHasDepthCamera ? 1 : 0, bHasSegmentationCamera ? 1 : 0);
    
    // Reset checkboxes if corresponding cameras aren't available
    if (!bHasRGBCamera)
    {
        bUseRGBCamera = false;
        RGBCameraCheckBox->SetIsChecked(ECheckBoxState::Unchecked);
    }
    
    if (!bHasDepthCamera)
    {
        bUseDepthCamera = false;
        DepthCameraCheckBox->SetIsChecked(ECheckBoxState::Unchecked);
    }
    
    if (!bHasSegmentationCamera)
    {
        bUseSegmentationCamera = false;
        SegmentationCameraCheckBox->SetIsChecked(ECheckBoxState::Unchecked);
    }

    UpdateActiveCameras();
}

// Update active camera components
void SVCCSimPanel::UpdateActiveCameras()
{
    if (!SelectedFlashPawn.IsValid())
    {
        return;
    }
    
    // Update RGB cameras
    TArray<URGBCameraComponent*> RGBCameras;
    SelectedFlashPawn->GetComponents<URGBCameraComponent>(RGBCameras);
    for (URGBCameraComponent* Camera : RGBCameras)
    {
        if (Camera)
        {
            Camera->SetActive(bUseRGBCamera);
            Camera->InitializeRenderTargets();
            Camera->SetCaptureComponent();
        }
    }
    
    // Update Depth cameras
    TArray<UDepthCameraComponent*> DepthCameras;
    SelectedFlashPawn->GetComponents<UDepthCameraComponent>(DepthCameras);
    for (UDepthCameraComponent* Camera : DepthCameras)
    {
        if (Camera)
        {
            Camera->SetActive(bUseDepthCamera);
        }
    }
    
    // Update Segmentation cameras
    TArray<USegmentationCameraComponent*> SegmentationCameras;
    SelectedFlashPawn->GetComponents<USegmentationCameraComponent>(SegmentationCameras);
    for (USegmentationCameraComponent* Camera : SegmentationCameras)
    {
        if (Camera)
        {
            Camera->SetActive(bUseSegmentationCamera);
        }
    }
    
    UE_LOG(LogTemp, Display, TEXT("Camera activation updated: RGB=%s, Depth=%s, Segmentation=%s"),
        bUseRGBCamera ? TEXT("On") : TEXT("Off"),
        bUseDepthCamera ? TEXT("On") : TEXT("Off"),
        bUseSegmentationCamera ? TEXT("On") : TEXT("Off"));
}

// Generate poses around target
FReply SVCCSimPanel::OnGeneratePosesClicked()
{
    GeneratePosesAroundTarget();
    return FReply::Handled();
}

void SVCCSimPanel::GeneratePosesAroundTarget()
{
    if (!SelectedFlashPawn.IsValid() || !SelectedTargetObject.IsValid())
    {
        UE_LOG(LogTemp, Warning,
            TEXT("Must select both a FlashPawn and a target object"));
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
    SelectedFlashPawn->SetPathPanel(Positions, Rotations);
    
    // Reset any ongoing auto-capture
    bAutoCaptureInProgress = false;
    GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
    
    UE_LOG(LogTemp, Warning, TEXT("Generated %d poses around target"), NumPoses);
}

// Capture image from current pose
FReply SVCCSimPanel::OnCaptureImagesClicked()
{
    CaptureImageFromCurrentPose();
    return FReply::Handled();
}

void SVCCSimPanel::SaveRGB(int32 PoseIndex, bool& bAnyCaptured)
{
    TArray<URGBCameraComponent*> RGBCameras;
    SelectedFlashPawn->GetComponents<URGBCameraComponent>(RGBCameras);
            
    for (int32 i = 0; i < RGBCameras.Num(); ++i)
    {
        URGBCameraComponent* Camera = RGBCameras[i];
        if (Camera && Camera->IsActive())
        {
            // Get camera index or use iterator index
            int32 CameraIndex = Camera->GetCameraIndex();
            if (CameraIndex < 0) CameraIndex = i;
                    
            // Filename for this camera
            FString Filename = SaveDirectory / FString::Printf(
                TEXT("RGB_Cam%02d_Pose%03d.png"), 
                CameraIndex, 
                PoseIndex
            );
                    
            // Capture the image
            Camera->CaptureRGBScene();
            FIntPoint Size = {Camera->GetImageSize().first, Camera->GetImageSize().second};
                    
            // Get image data and save asynchronously
            Camera->AsyncGetRGBImageData(
                [Filename, Size](const TArray<FColor>& ImageData)
                {
                    (new FAutoDeleteAsyncTask<FAsyncImageSaveTask>(ImageData, Size, Filename))
                    ->StartBackgroundTask();
                });
                    
            bAnyCaptured = true;
        }
    }
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
        // Pose index for filename
        int32 PoseIndex = SelectedFlashPawn->GetCurrentIndex();
        
        // Track if any cameras were captured
        bool bAnyCaptured = false;
        
        // Capture with RGB cameras if enabled
        if (bUseRGBCamera && bHasRGBCamera)
        {
            SaveRGB(PoseIndex, bAnyCaptured);
        }
        
        // Capture with Depth cameras if enabled
        if (bUseDepthCamera && bHasDepthCamera)
        {
            // Implementation for depth cameras
            // Similar to RGB but with depth-specific methods
            bAnyCaptured = true;
        }
        
        // Capture with Segmentation cameras if enabled
        if (bUseSegmentationCamera && bHasSegmentationCamera)
        {
            // Implementation for segmentation cameras
            // Similar to RGB but with segmentation-specific methods
            bAnyCaptured = true;
        }
        
        // Log if no images were captured
        if (!bAnyCaptured)
        {
            UE_LOG(LogTemp, Warning, TEXT("No images captured. "
                                          "Ensure cameras are enabled."));
        }
        
        // Move to next position
        SelectedFlashPawn->MoveToNext();

        SaveDirectory.Empty();
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("FlashPawn not ready for capture. "
                                      "Wait for it to reach position."));
    }
}

// Start auto-capture
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

    SelectedFlashPawn->MoveTo(0);
    
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
                if (SelectedFlashPawn->GetCurrentIndex() == NumPoses - 1)
                {
                    SaveDirectory.Empty(); // Reset for next capture session
                    UE_LOG(LogTemp, Display, TEXT("Auto-capture completed"));
                    bAutoCaptureInProgress = false;
                    GEditor->GetTimerManager()->ClearTimer(AutoCaptureTimerHandle);
                }
                else
                {
                    // Move to the next pose
                    SelectedFlashPawn->MoveForward();
                }
            }
        },
        0.01f, // Check every 0.5 seconds
        true  // Looping
    );
}

// Helper to get a timestamped filename
FString SVCCSimPanel::GetTimestampedFilename()
{
    FDateTime Now = FDateTime::Now();
    return FString::Printf(TEXT("%04d-%02d-%02d_%02d-%02d-%02d"),
        Now.GetYear(), Now.GetMonth(), Now.GetDay(),
        Now.GetHour(), Now.GetMinute(), Now.GetSecond());
}

namespace FVCCSimPanelFactory
{
    const FName TabId = FName("VCCSimPanel");
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