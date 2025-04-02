#pragma once

#include "CoreMinimal.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/SCompoundWidget.h"
#include "Editor/PropertyEditor/Public/IDetailsView.h"
#include "Pawns/FlashPawn.h"

class VCCSIM_API SVCCSimPanel final : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SVCCSimPanel) {}
    SLATE_END_ARGS()

    ~SVCCSimPanel();

    void Construct(const FArguments& InArgs);
    
    // Update signature to match UE's selection event
    void OnSelectionChanged(UObject* Object);

private:
    // UI Elements - Removed combo boxes, added selection state toggles
    TSharedPtr<FSlateDynamicImageBrush> VCCLogoBrush;
    TSharedPtr<FSlateDynamicImageBrush> SZULogoBrush;
    
    TSharedPtr<class STextBlock> SelectedFlashPawnText;
    TSharedPtr<class STextBlock> SelectedTargetObjectText;
    TSharedPtr<class SNumericEntryBox<int32>> NumPosesSpinBox;
    TSharedPtr<class SNumericEntryBox<float>> RadiusSpinBox;
    TSharedPtr<class SNumericEntryBox<float>> HeightOffsetSpinBox;
    
    // Camera availability indicators
    TSharedPtr<class STextBlock> RGBCameraAvailableText;
    TSharedPtr<class STextBlock> DepthCameraAvailableText;
    TSharedPtr<class STextBlock> SegmentationCameraAvailableText;
    
    // Camera activation checkboxes
    TSharedPtr<class SCheckBox> RGBCameraCheckBox;
    TSharedPtr<class SCheckBox> DepthCameraCheckBox;
    TSharedPtr<class SCheckBox> SegmentationCameraCheckBox;
    
    // Selection state toggles
    TSharedPtr<class SCheckBox> SelectFlashPawnToggle;
    TSharedPtr<class SCheckBox> SelectTargetToggle;
    bool bSelectingFlashPawn = false;
    bool bSelectingTarget = false;
    
    // Selected objects
    TWeakObjectPtr<AFlashPawn> SelectedFlashPawn;
    TWeakObjectPtr<AActor> SelectedTargetObject;
    
    // Configuration
    int32 NumPoses = 8;
    float Radius = 500.0f;
    float HeightOffset = 0.0f;
    FString SaveDirectory;
    
    // Camera settings
    bool bUseRGBCamera = true;
    bool bUseDepthCamera = false;
    bool bUseSegmentationCamera = false;
    
    // Available cameras on current FlashPawn
    bool bHasRGBCamera = false;
    bool bHasDepthCamera = false;
    bool bHasSegmentationCamera = false;
    
    // Auto-capture state
    bool bAutoCaptureInProgress = false;
    FTimerHandle AutoCaptureTimerHandle;
    
    // UI creation helpers
    TSharedRef<SWidget> CreatePawnSelectPanel();
    TSharedRef<SWidget> CreateTargetSelectPanel();
    TSharedRef<SWidget> CreatePoseConfigPanel();
    TSharedRef<SWidget> CreateCapturePanel();
    TSharedRef<SWidget> CreateCameraSelectPanel();
    
    // UI callbacks
    void OnSelectFlashPawnToggleChanged(ECheckBoxState NewState);
    void OnSelectTargetToggleChanged(ECheckBoxState NewState);
    
    void OnRGBCameraCheckboxChanged(ECheckBoxState NewState);
    void OnDepthCameraCheckboxChanged(ECheckBoxState NewState);
    void OnSegmentationCameraCheckboxChanged(ECheckBoxState NewState);
    
    FReply OnGeneratePosesClicked();
    FReply OnCaptureImagesClicked();
    void SaveRGB(int32 PoseIndex, bool& bAnyCaptured);
    void StartAutoCapture();
    
    // Helper functions
    void GeneratePosesAroundTarget();
    void CaptureImageFromCurrentPose();
    void CheckCameraComponents();
    void UpdateActiveCameras();
    static FString GetTimestampedFilename();
};

namespace FVCCSimPanelFactory
{
    extern const FName TabId;
    void RegisterTabSpawner(FTabManager& TabManager);
}