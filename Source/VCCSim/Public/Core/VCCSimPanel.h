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
    
    void RefreshProperties();

private:
    // UI Elements
    TSharedPtr<class SComboBox<TWeakObjectPtr<AActor>>> FlashPawnComboBox;
    TSharedPtr<class SComboBox<TWeakObjectPtr<AActor>>> TargetObjectComboBox;
    TSharedPtr<class SNumericEntryBox<int32>> NumPosesSpinBox;
    TSharedPtr<class SNumericEntryBox<float>> RadiusSpinBox;
    TSharedPtr<class SNumericEntryBox<float>> HeightOffsetSpinBox;
    
    // Selected objects
    TWeakObjectPtr<AFlashPawn> SelectedFlashPawn;
    TWeakObjectPtr<AActor> SelectedTargetObject;
    
    // Data for combo boxes
    TArray<TWeakObjectPtr<AActor>> FlashPawnOptions;
    TArray<TWeakObjectPtr<AActor>> TargetObjectOptions;
    
    // Configuration
    int32 NumPoses = 8;
    float Radius = 500.0f;
    float HeightOffset = 0.0f;
    FString SaveDirectory;
    
    // Auto-capture state
    bool bAutoCaptureInProgress = false;
    FTimerHandle AutoCaptureTimerHandle;
    
    // UI creation helpers
    TSharedRef<SWidget> CreatePawnSelectPanel();
    TSharedRef<SWidget> CreateTargetSelectPanel();
    TSharedRef<SWidget> CreatePoseConfigPanel();
    TSharedRef<SWidget> CreateCapturePanel();
    
    // UI callbacks
    void RefreshActorLists();
    TSharedRef<SWidget> GenerateFlashPawnComboItem(TWeakObjectPtr<AActor> InItem);
    FText GetFlashPawnComboText() const;
    void OnFlashPawnSelectionChanged(TWeakObjectPtr<AActor> NewSelection, ESelectInfo::Type SelectInfo);
    
    TSharedRef<SWidget> GenerateTargetObjectComboItem(TWeakObjectPtr<AActor> InItem);
    FText GetTargetObjectComboText() const;
    void OnTargetObjectSelectionChanged(TWeakObjectPtr<AActor> NewSelection, ESelectInfo::Type SelectInfo);
    
    FReply OnGeneratePosesClicked();
    FReply OnCaptureImagesClicked();
    void StartAutoCapture();
    
    // Helper functions
    void GeneratePosesAroundTarget();
    void CaptureImageFromCurrentPose();
    FString GetTimestampedFilename() const;
};

namespace FVCCSimPanelFactory
{
    extern const FName TabId;
    void RegisterTabSpawner(FTabManager& TabManager);
}