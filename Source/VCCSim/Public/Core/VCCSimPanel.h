#pragma once

#include "CoreMinimal.h"
#include "Widgets/Docking/SDockTab.h"
#include "Widgets/SCompoundWidget.h"
#include "Editor/PropertyEditor/Public/IDetailsView.h"

class AFlashPawn;
class AVCCSimPath;
class USplineMeshComponent;

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
    TSharedPtr<class SNumericEntryBox<float>> VerticalGapSpinBox;
    
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
    int32 NumPoses = 50;
    float Radius = 500.0f;
    float HeightOffset = 0.0f;
    float VerticalGap = 50.0f;
    FString SaveDirectory;
    
    // TOptional attributes for SpinBox values
    TOptional<int32> NumPosesValue;
    TOptional<float> RadiusValue;
    TOptional<float> HeightOffsetValue;
    TOptional<float> VerticalGapValue;
    
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
    TSharedRef<SWidget> CreateCameraSelectPanel();
    TSharedRef<SWidget> CreateTargetSelectPanel();
    TSharedRef<SWidget> CreatePoseConfigPanel();
    TSharedRef<SWidget> CreateCapturePanel();
    
    // UI callbacks
    void OnSelectFlashPawnToggleChanged(ECheckBoxState NewState);
    void OnSelectTargetToggleChanged(ECheckBoxState NewState);
    
    void OnRGBCameraCheckboxChanged(ECheckBoxState NewState);
    void OnDepthCameraCheckboxChanged(ECheckBoxState NewState);
    void OnSegmentationCameraCheckboxChanged(ECheckBoxState NewState);
    
    FReply OnGeneratePosesClicked();
    FReply OnCaptureImagesClicked();
    void SaveRGB(int32 PoseIndex, bool& bAnyCaptured);
    void SaveDepth(int32 PoseIndex, bool& bAnyCaptured);
    void SaveSeg(int32 PoseIndex, bool& bAnyCaptured);
    void StartAutoCapture();
    TSharedPtr<std::atomic<int32>> JobNum;
    
    // Helper functions
    void GeneratePosesAroundTarget();
    void CaptureImageFromCurrentPose();
    void CheckCameraComponents();
    void UpdateActiveCameras();
    static FString GetTimestampedFilename();
    
    // New pose file functions
    void LoadPredefinedPose();
    void SaveGeneratedPose();
    FReply OnLoadPoseClicked();
    FReply OnSavePoseClicked();
    // Add these to your SVCCSimPanel.h file under the private section

    TSharedRef<SWidget> CreateSectionHeader(const FString& Title);
    TSharedRef<SWidget> CreateSectionContent(TSharedRef<SWidget> Content);
    TSharedRef<SWidget> CreatePropertyRow(const FString& Label, TSharedRef<SWidget> Content);

    bool bPathVisualized = false;
    bool bPathNeedsUpdate = true;
    TWeakObjectPtr<AActor> PathVisualizationActor;
    TSharedPtr<class SButton> VisualizePathButton;
    FReply OnTogglePathVisualizationClicked();
    void UpdatePathVisualization();
    void ShowPathVisualization();
    void HidePathVisualization();
};

namespace FVCCSimPanelFactory
{
    extern const FName TabId;
    void RegisterTabSpawner(FTabManager& TabManager);
}