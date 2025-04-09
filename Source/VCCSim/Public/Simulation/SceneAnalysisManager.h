#pragma once

#include "CoreMinimal.h"
#include "DataType/DataMesh.h"
#include "ProceduralMeshComponent.h"
#include "SceneAnalysisManager.generated.h"

class URGBCameraComponent;

struct FCoverageData
{
    float CoveragePercentage;
    TSet<int32> VisibleMeshIDs;
    TArray<FVector> VisiblePoints;
    int32 TotalVisibleTriangles;
};

struct FCoverageCell
{
    int32 TotalPoints;
    int32 VisiblePoints;
    float Coverage;
    
    FCoverageCell() : TotalPoints(0), VisiblePoints(0), Coverage(0.0f) {}
};

UCLASS(BlueprintType, Blueprintable)
class VCCSIM_API ASceneAnalysisManager : public AActor
{
    GENERATED_BODY()

public:
    // Base functions
    ASceneAnalysisManager();
    bool Initialize(UWorld* InWorld, FString&& Path);
    void ScanScene();
    void RegisterCamera(URGBCameraComponent* CameraComponent);

    // Coverage
    float GetTotalCoveragePercentage() const { return CurrentCoveragePercentage;}
    void ResetCoverage();
    FCoverageData ComputeCoverage(
        const TArray<FTransform>& CameraTransforms, const FString& CameraName);
    FCoverageData ComputeCoverage(
        const FTransform& CameraTransform, const FString& CameraName);
    
    // Coverage Grid Visualization - New Approach
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis|Coverage")
    void InitializeCoverageVisualization();
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis|Coverage")
    void VisualizeCoverage(bool bShow);
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis|Coverage")
    void ClearCoverageVisualization();
    
    // Coverage Grid Settings
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage")
    UProceduralMeshComponent* CoverageVisualizationMesh;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage")
    UMaterialInterface* CoverageMaterial;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage")
    float CoverageGridResolution = 50.0f;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage")
    int SamplingDensity = 1;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage")
    bool bUseVertexSampling = true;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis|Coverage", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float VisualizationThreshold = 0.f;
    
    // Safe zone
    void GenerateSafeZone(const float& SafeDistance, const float& SafeHeight);
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis | SafeZone")
    float GridResolution = 100.f;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis | SafeZone")
    UInstancedStaticMeshComponent* SafeZoneInstancedMesh;
    UPROPERTY(EditAnywhere, Category = "SceneAnalysis | SafeZone")
    UMaterialInterface* SafeZoneMaterial;
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis | SafeZone")
    void InitializeSafeZoneVisualization();
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis | SafeZone")
    void VisualizeSafeZone(bool Vis);
    UFUNCTION(BlueprintCallable, Category = "SceneAnalysis | SafeZone")
    void ClearSafeZoneVisualization();

    // Helper functions
    FMeshInfo GetMeshInfo(int32 MeshID) const;
    TArray<FMeshInfo> GetAllMeshInfo() const;
    static void ExtractMeshData(UStaticMeshComponent* MeshComponent,
    FMeshInfo& OutMeshInfo);
    FIntVector WorldToGridCoordinates(const FVector& WorldPos) const;
    
    /* ----------------------------- Test ----------------------------- */
    FString LogPath;
    void ExportMeshesToPly();
    FString GeneratePlyContent(const FMeshInfo& MeshInfo);
    void VisualizeSceneMeshes(float Duration, bool bShowWireframe,
        bool bShowVertices, float VertexSize);
    void VisualizeSampledPoints(float Duration, float VertexSize);

private:
    void ConstructFrustum(FConvexVolume& OutFrustum,
        const FTransform& CameraPose, const FMatrix44f& CameraIntrinsic);
    bool IsPointVisibleFromCamera(const FVector& Point,
        const FTransform& CameraPose) const;
    TArray<FVector> SamplePointsOnMesh(const FMeshInfo& MeshInfo);

    void InitializeCoverageGrid();
    void UpdateCoverageGrid();
    void CreateCoverageMesh();

private:
    UPROPERTY()
    UWorld* World;
    
    TMap<FString, FMatrix44f> CameraIntrinsics;
    
    TArray<FMeshInfo> SceneMeshes;
    int32 TotalPointsInScene;
    int32 TotalTrianglesInScene;
    
    TSet<int32> CurrentlyVisibleMeshIDs;

    // Coverage
    TMap<FVector, bool> CoverageMap;
    float CurrentCoveragePercentage;
    bool bCoverageVisualizationDirty = false;
    TArray<FVector> VisiblePoints;
    TArray<FVector> InvisiblePoints;

    // Coverage Grid
    TMap<FIntVector, FCoverageCell> CoverageGrid;
    FVector CoverageGridOrigin;
    FVector CoverageGridSize;
    bool bCoverageGridInitialized = false;

    // Safe zone
    UPROPERTY()
    FBox ExpandedSceneBounds;
    TArray<TArray<TArray<bool>>> SafeZoneGrid;
    bool bSafeZoneDirty = false;
};