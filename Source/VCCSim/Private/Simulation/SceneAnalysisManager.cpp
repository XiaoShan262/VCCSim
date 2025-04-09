#include "Simulation/SceneAnalysisManager.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "TimerManager.h"
#include "Sensors/CameraSensor.h"
#include "EngineUtils.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
#include "DrawDebugHelpers.h"
#include "MaterialDomain.h"

ASceneAnalysisManager::ASceneAnalysisManager()
{
    World = nullptr;
    TotalPointsInScene = 0;
    TotalTrianglesInScene = 0;
    CurrentCoveragePercentage = 0.0f;
    LogPath = FPaths::ProjectLogDir();
    SafeZoneMaterial = nullptr;
    SafeZoneVisualizationMesh = nullptr;
    CoverageVisualizationMesh = nullptr;
    CoverageMaterial = nullptr;

    // Set no collision and no tick
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);
}

bool ASceneAnalysisManager::Initialize(UWorld* InWorld, FString&& Path)
{
    if (!InWorld)
        return false;
    
    World = InWorld;
    LogPath = std::move(Path) + "/SceneAnalysisLog";
    return true;
}

void ASceneAnalysisManager::ScanScene()
{
    if (!World)
        return;
    
    // Clear previous data
    SceneMeshes.Empty();
    TotalPointsInScene = 0;
    TotalTrianglesInScene = 0;
    
    // Iterate only through StaticMeshActor objects in the world
    for (TActorIterator<AStaticMeshActor> ActorItr(World); ActorItr; ++ActorItr)
    {
        AStaticMeshActor* StaticMeshActor = *ActorItr;
        UStaticMeshComponent* MeshComp = StaticMeshActor->GetStaticMeshComponent();
        
        if (MeshComp && MeshComp->GetStaticMesh())
        {
            FMeshInfo MeshInfo;
            ExtractMeshData(MeshComp, MeshInfo);
            SceneMeshes.Add(MeshInfo);
            
            TotalTrianglesInScene += MeshInfo.NumTriangles;
            TotalPointsInScene += MeshInfo.NumVertices;
        }
    }
    
    ResetCoverage();
}

void ASceneAnalysisManager::RegisterCamera(URGBCameraComponent* CameraComponent)
{
    CameraIntrinsics.Add(CameraComponent->CameraName,
    CameraComponent->GetCameraIntrinsics());

    // CameraComponent->OnKeyPointCaptured.BindUFunction(
    //     this, "");
}

FMeshInfo ASceneAnalysisManager::GetMeshInfo(int32 MeshID) const
{
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        if (MeshInfo.MeshID == MeshID)
            return MeshInfo;
    }
    
    return FMeshInfo();
}

TArray<FMeshInfo> ASceneAnalysisManager::GetAllMeshInfo() const
{
    return SceneMeshes;
}

FCoverageData ASceneAnalysisManager::ComputeCoverage(
    const TArray<FTransform>& CameraTransforms, const FString& CameraName)
{
    FCoverageData CoverageData;
    CoverageData.CoveragePercentage = 0.0f;
    CoverageData.TotalVisibleTriangles = 0;
    
    if (!World || CameraTransforms.Num() == 0 || CoverageMap.Num() == 0)
        return CoverageData;
    
    // Get camera intrinsic for specified camera name
    FMatrix44f CameraIntrinsic;
    
    if (CameraIntrinsics.Contains(CameraName))
    {
        CameraIntrinsic = CameraIntrinsics[CameraName];
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("ComputeCoverage: No intrinsics "
                                     "found for camera %s"), *CameraName);
    }
    
    // Reset visibility of all points
    VisiblePoints.Empty();
    InvisiblePoints.Empty();
    
    // Reset all points to invisible first
    for (auto& Pair : CoverageMap)
    {
        Pair.Value = false;
    }
    
    // For each camera, check which points are visible
    for (const FTransform& CameraTransform : CameraTransforms)
    {
        // Construct frustum for this camera
        FConvexVolume Frustum;
        ConstructFrustum(Frustum, CameraTransform, CameraIntrinsic);

        // OPTIMIZATION: Use parallel processing for large point sets
        if (CoverageMap.Num() > 10000)
        {
            // Create thread-safe data structures
            FCriticalSection CriticalSection;
            ParallelFor(CoverageMap.Num(), [&](int32 Index)
            {
                auto It = CoverageMap.CreateIterator();
                for (int32 i = 0; i < Index; ++i)
                {
                    ++It;
                }
                
                const FVector& Point = It.Key();
                bool& bIsVisible = It.Value();
                
                // Skip points that are already marked as visible
                if (bIsVisible)
                    return;
                
                // OPTIMIZATION: Early frustum culling
                bool bInFrustum = true;
                for (const FPlane& Plane : Frustum.Planes)
                {
                    if (Plane.PlaneDot(Point) < 0.0f)
                    {
                        bInFrustum = false;
                        break;
                    }
                }
                
                // Only perform expensive visibility check if in frustum
                if (bInFrustum && IsPointVisibleFromCamera(Point, CameraTransform))
                {
                    bIsVisible = true;
                }
            });
        }
        else
        {
            // Original sequential processing for smaller point sets
            for (auto& Pair : CoverageMap)
            {
                const FVector& Point = Pair.Key;
                bool& bIsVisible = Pair.Value;
                
                // Skip points that are already marked as visible
                if (bIsVisible)
                    continue;
                
                // OPTIMIZATION: Early frustum culling
                bool bInFrustum = true;
                for (const FPlane& Plane : Frustum.Planes)
                {
                    if (Plane.PlaneDot(Point) < 0.0f)
                    {
                        bInFrustum = false;
                        break;
                    }
                }
                
                // Only perform expensive visibility check if in frustum
                if (bInFrustum && IsPointVisibleFromCamera(Point, CameraTransform))
                {
                    bIsVisible = true;
                }
            }
        }
    }
    
    // Process visibility results
    for (const auto& Pair : CoverageMap)
    {
        if (Pair.Value)
        {
            VisiblePoints.Add(Pair.Key);
        }
        else
        {
            InvisiblePoints.Add(Pair.Key);
        }
    }
    
    // Calculate coverage percentage
    int32 TotalPoints = CoverageMap.Num();
    int32 VisiblePointCount = VisiblePoints.Num();
    
    // Identify visible meshes using spatial partitioning for efficiency
    TSet<int32> VisibleMeshIDs;
    for (const FVector& Point : VisiblePoints)
    {
        for (const FMeshInfo& MeshInfo : SceneMeshes)
        {
            if (MeshInfo.Bounds.GetBox().IsInsideOrOn(Point))
            {
                VisibleMeshIDs.Add(MeshInfo.MeshID);
                break;
            }
        }
    }
    
    // Calculate coverage percentage
    float CoveragePercentage = TotalPoints > 0 ? (float)VisiblePointCount /
        (float)TotalPoints * 100.0f : 0.0f;
    
    // Update class members
    CurrentlyVisibleMeshIDs = VisibleMeshIDs;
    CurrentCoveragePercentage = CoveragePercentage;
    bCoverageVisualizationDirty = true;
    
    // Populate return data structure
    CoverageData.CoveragePercentage = CoveragePercentage;
    CoverageData.VisibleMeshIDs = VisibleMeshIDs;
    CoverageData.VisiblePoints = VisiblePoints;
    
    // Calculate total visible triangles (approx based on visible points)
    int32 VisibleTriangles = 0;
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        if (VisibleMeshIDs.Contains(MeshInfo.MeshID))
        {
            // Approximate visible triangles based on visible sample points from this mesh
            float MeshVisiblePointRatio = 0.0f;
            int32 MeshTotalPoints = 0;
            int32 MeshVisiblePoints = 0;
            
            for (const auto& Pair : CoverageMap)
            {
                if (MeshInfo.Bounds.GetBox().IsInsideOrOn(Pair.Key))
                {
                    MeshTotalPoints++;
                    if (Pair.Value)
                        MeshVisiblePoints++;
                }
            }
            
            if (MeshTotalPoints > 0)
            {
                MeshVisiblePointRatio = (float)MeshVisiblePoints / (float)MeshTotalPoints;
                VisibleTriangles += FMath::RoundToInt(MeshInfo.NumTriangles * MeshVisiblePointRatio);
            }
        }
    }
    
    CoverageData.TotalVisibleTriangles = VisibleTriangles;
    
    UE_LOG(LogTemp, Display, TEXT("Coverage computed for camera %s: %.2f%% of points visible (%d/%d), %d visible meshes, ~%d visible triangles"),
        *CameraName, CoveragePercentage, VisiblePointCount, TotalPoints, VisibleMeshIDs.Num(), VisibleTriangles);

    if (bGridInitialized)
    {
        UpdateCoverageGrid();
    }
    
    return CoverageData;
}

FCoverageData ASceneAnalysisManager::ComputeCoverage(
    const FTransform& CameraTransform, const FString& CameraName)
{
    // Create array with single transform and call the multi-transform version
    TArray<FTransform> CameraTransforms;
    CameraTransforms.Add(CameraTransform);
    return ComputeCoverage(CameraTransforms, CameraName);
}

void ASceneAnalysisManager::InitializeCoverageVisualization()
{
    if (!World)
        return;
    
    // Initialize the procedural mesh component if it doesn't exist
    if (!CoverageVisualizationMesh)
    {
        CoverageVisualizationMesh = NewObject<UProceduralMeshComponent>(this);
        CoverageVisualizationMesh->RegisterComponent();
        CoverageVisualizationMesh->SetMobility(EComponentMobility::Movable);
        CoverageVisualizationMesh->AttachToComponent(GetRootComponent(), 
                                                   FAttachmentTransformRules::KeepWorldTransform);
        CoverageVisualizationMesh->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
    }
    
    // Load or create the coverage material if not already set
    if (!CoverageMaterial)
    {
        // Try to load the coverage material
        CoverageMaterial = LoadObject<UMaterialInterface>(nullptr, 
            TEXT("/Script/Engine.Material'/VCCSim/Materials/M_Coverage.M_Coverage'"));
        
        // Only log error if material failed to load
        if (!CoverageMaterial)
        {
            UE_LOG(LogTemp, Error, TEXT("InitializeCoverageVisualization: "
                                       "Failed to load coverage material."));
        }
    }
}

void ASceneAnalysisManager::VisualizeCoverage(bool bShow)
{
    if (!World)
        return;
    
    // Check if we have coverage data
    if (!bGridInitialized)
    {
        UE_LOG(LogTemp, Warning, TEXT("VisualizeCoverage: No coverage grid initialized"));
        return;
    }
    
    // If not showing, clear visualization and return
    if (!bShow)
    {
        if (CoverageVisualizationMesh)
        {
            CoverageVisualizationMesh->SetVisibility(false);
        }
        return;
    }
    
    // Initialize visualization components if needed
    InitializeCoverageVisualization();
    
    // If mesh component failed to initialize, return
    if (!CoverageVisualizationMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("VisualizeCoverage: Coverage "
                                    "mesh component not initialized"));
        return;
    }
    
    // Only update mesh if dirty or visibility is changing
    if (bCoverageVisualizationDirty)
    {
        CreateCoverageMesh();
        bCoverageVisualizationDirty = false;
    }
    
    // Set visibility
    CoverageVisualizationMesh->SetVisibility(true);
}

void ASceneAnalysisManager::ClearCoverageVisualization()
{
    if (CoverageVisualizationMesh)
    {
        CoverageVisualizationMesh->ClearAllMeshSections();
        CoverageVisualizationMesh->SetVisibility(false);
    }
}

// Update the ResetCoverage function to initialize the point arrays
void ASceneAnalysisManager::ResetCoverage()
{
    CoverageMap.Empty();
    CurrentlyVisibleMeshIDs.Empty();
    CurrentCoveragePercentage = 0.0f;
    VisiblePoints.Empty();
    InvisiblePoints.Empty();
    
    // Initialize coverage map with all points set to not visible
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        TArray<FVector> SampledPoints = SamplePointsOnMesh(MeshInfo);
        for (const FVector& Point : SampledPoints)
        {
            CoverageMap.Add(Point, false);
            InvisiblePoints.Add(Point);
        }
    }
    
    // Initialize the unified grid
    InitializeUnifiedGrid();
    
    bCoverageVisualizationDirty = true;
}

void ASceneAnalysisManager::ConstructFrustum(
    FConvexVolume& OutFrustum, const FTransform& CameraPose, const FMatrix44f& CameraIntrinsic)
{
    // Extract camera parameters
    const float fx = CameraIntrinsic.M[0][0];
    const float fy = CameraIntrinsic.M[1][1];
    const float cx = CameraIntrinsic.M[0][2];
    const float cy = CameraIntrinsic.M[1][2];
    
    // Calculate image width and height from principal points (assuming centered)
    const float width = cx * 2.0f;  // Width is twice the x principal point
    const float height = cy * 2.0f; // Height is twice the y principal point
    
    // Calculate FOV correctly
    float HorizontalFOV = (fx > 0.0f) ? 2.0f * FMath::Atan(width / (2.0f * fx))
    : FMath::DegreesToRadians(90.0f);
    float VerticalFOV = (fy > 0.0f) ? 2.0f * FMath::Atan(height / (2.0f * fy))
    : FMath::DegreesToRadians(60.0f);
    float AspectRatio = width / height;
    
    // If values are extreme, use reasonable defaults
    if (FMath::IsNaN(HorizontalFOV) || HorizontalFOV < FMath::DegreesToRadians(1.0f)) {
        UE_LOG(LogTemp, Warning, TEXT("FOV calculation failed - using default 90°"));
        HorizontalFOV = FMath::DegreesToRadians(90.0f);
    }
    
    // Create frustum planes
    const FVector ForwardVector = CameraPose.GetRotation().GetForwardVector();
    const FVector RightVector = CameraPose.GetRotation().GetRightVector();
    const FVector UpVector = CameraPose.GetRotation().GetUpVector();
    const FVector Position = CameraPose.GetLocation();
    
    constexpr float NearPlaneDistance = 10.0f;
    constexpr float FarPlaneDistance = 5000.0f;
    
    // Calculate frustum corners
    const float HalfVFOV = VerticalFOV * 0.5f;

    const float FarHeight = FarPlaneDistance * FMath::Tan(HalfVFOV);
    const float FarWidth = FarHeight * AspectRatio;
    
    // Near and far plane centers
    const FVector NearCenter = Position + ForwardVector * NearPlaneDistance;
    const FVector FarCenter = Position + ForwardVector * FarPlaneDistance;
    
    // Far plane corners
    const FVector FarTopLeft = FarCenter + UpVector * FarHeight - RightVector * FarWidth;
    const FVector FarTopRight = FarCenter + UpVector * FarHeight + RightVector * FarWidth;
    const FVector FarBottomLeft = FarCenter - UpVector * FarHeight - RightVector * FarWidth;
    const FVector FarBottomRight = FarCenter - UpVector * FarHeight + RightVector * FarWidth;

    // Create frustum planes
    OutFrustum.Planes.Empty(6);
    
    // Near plane (normal points backward)
    OutFrustum.Planes.Add(FPlane(NearCenter, ForwardVector));
    
    // Far plane (normal points forward)
    OutFrustum.Planes.Add(FPlane(FarCenter, -ForwardVector));
    
    // Left plane
    FVector LeftNormal = -FVector::CrossProduct(
        FarBottomLeft - Position, FarTopLeft - Position).GetSafeNormal();
    OutFrustum.Planes.Add(FPlane(Position, LeftNormal));

    // Right plane
    FVector RightNormal = -FVector::CrossProduct(
        FarTopRight - Position, FarBottomRight - Position).GetSafeNormal();
    OutFrustum.Planes.Add(FPlane(Position, RightNormal));

    // Top plane
    FVector TopNormal = -FVector::CrossProduct(
        FarTopLeft - Position, FarTopRight - Position).GetSafeNormal();
    OutFrustum.Planes.Add(FPlane(Position, TopNormal));

    // Bottom plane
    FVector BottomNormal = -FVector::CrossProduct(
        FarBottomRight - Position, FarBottomLeft - Position).GetSafeNormal();
    OutFrustum.Planes.Add(FPlane(Position, BottomNormal));
}

void ASceneAnalysisManager::ExtractMeshData(
    UStaticMeshComponent* MeshComponent, FMeshInfo& OutMeshInfo)
{
    if (!MeshComponent || !MeshComponent->GetStaticMesh())
        return;
    
    UStaticMesh* StaticMesh = MeshComponent->GetStaticMesh();
    
    // Basic mesh info
    OutMeshInfo.MeshID = MeshComponent->GetUniqueID();
    OutMeshInfo.MeshName = MeshComponent->GetName();
    OutMeshInfo.Mesh = StaticMesh;
    OutMeshInfo.Transform = MeshComponent->GetComponentTransform();
    OutMeshInfo.Bounds = MeshComponent->Bounds;
    OutMeshInfo.bIsVisible = MeshComponent->IsVisible();
    
    // Get mesh data
    if (StaticMesh->GetRenderData() && StaticMesh->GetRenderData()->LODResources.Num() > 0)
    {
        const FStaticMeshLODResources& LODModel = StaticMesh->GetRenderData()->LODResources[0];
        
        // Get vertices and indices
        OutMeshInfo.NumVertices = LODModel.VertexBuffers.PositionVertexBuffer.GetNumVertices();
        OutMeshInfo.NumTriangles = LODModel.IndexBuffer.GetNumIndices() / 3;
        
        // Extract vertex positions
        OutMeshInfo.VertexPositions.Reserve(OutMeshInfo.NumVertices);
        for (int32 VertIdx = 0; VertIdx < OutMeshInfo.NumVertices; ++VertIdx)
        {
            // Get the FVector3f from the vertex buffer
            FVector3f VertexPos3f = LODModel.VertexBuffers.
            PositionVertexBuffer.VertexPosition(VertIdx);
            
            // Convert to FVector (explicit conversion)
            FVector VertexPos(VertexPos3f.X, VertexPos3f.Y, VertexPos3f.Z);
            
            // Transform to world space
            VertexPos = OutMeshInfo.Transform.TransformPosition(VertexPos);
            OutMeshInfo.VertexPositions.Add(VertexPos);
        }
        
        // Extract indices
        OutMeshInfo.Indices.Reserve(LODModel.IndexBuffer.GetNumIndices());
        for (int32 IndexIdx = 0; IndexIdx < LODModel.IndexBuffer.GetNumIndices(); ++IndexIdx)
        {
            OutMeshInfo.Indices.Add(LODModel.IndexBuffer.GetIndex(IndexIdx));
        }
    }
}

FIntVector ASceneAnalysisManager::WorldToGridCoordinates(const FVector& WorldPos) const
{
    return FIntVector(
        FMath::FloorToInt((WorldPos.X - GridOrigin.X) / GridResolution),
        FMath::FloorToInt((WorldPos.Y - GridOrigin.Y) / GridResolution),
        FMath::FloorToInt((WorldPos.Z - GridOrigin.Z) / GridResolution)
    );
}

void ASceneAnalysisManager::InitializeUnifiedGrid()
{
    if (!World || SceneMeshes.Num() == 0)
        return;
    
    // Calculate bounds of the scene
    FBox SceneBounds(EForceInit::ForceInit);
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        SceneBounds += MeshInfo.Bounds.GetBox();
    }
    
    // Expand bounds slightly
    SceneBounds = SceneBounds.ExpandBy(GridResolution * 2.0f);
    ExpandedSceneBounds = SceneBounds;
    
    // Store grid origin
    GridOrigin = SceneBounds.Min;
    
    FVector BoundsSize = SceneBounds.GetSize();
    int32 GridSizeX = FMath::Max(1, FMath::CeilToInt(BoundsSize.X / GridResolution));
    int32 GridSizeY = FMath::Max(1, FMath::CeilToInt(BoundsSize.Y / GridResolution));
    int32 GridSizeZ = FMath::Max(1, FMath::CeilToInt(BoundsSize.Z / GridResolution));
    
    GridSize = FVector(GridSizeX, GridSizeY, GridSizeZ);
    
    // Clear the unified grid (sparse structure)
    UnifiedGrid.Empty();
    
    // Pre-populate the grid with cells that contain sampled points
    for (const auto& Pair : CoverageMap)
    {
        const FVector& Point = Pair.Key;
        FIntVector GridCoords = WorldToGridCoordinates(Point);
        
        // Add cell with no coverage yet
        FUnifiedGridCell& Cell = UnifiedGrid.FindOrAdd(GridCoords);
        Cell.TotalPoints++;
        Cell.bIsSafe = false; // Default to safe
    }
    
    bGridInitialized = true;
    
    UE_LOG(LogTemp, Display, TEXT("Unified grid initialized: "
                                  "theoretical grid %dx%dx%d, actual populated cells: %d"), 
           GridSizeX, GridSizeY, GridSizeZ, UnifiedGrid.Num());
}

void ASceneAnalysisManager::GenerateSafeZone(
    const float& SafeDistance, const float& SafeHeight)
{
    // Try to find a valid world
    if (GEngine)
    {
        for (const FWorldContext& Context : GEngine->GetWorldContexts())
        {
            if (Context.WorldType == EWorldType::Game || Context.WorldType == EWorldType::PIE)
            {
                World = Context.World();
            }
        }
    }
    
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("GenerateSafeZone: No valid World found"));
        return;
    }
    
    // Get scene mesh data
    if (SceneMeshes.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("GenerateSafeZone: No meshes in scene"));
        return;
    }
    
    // Initialize grid if not done already
    if (!bGridInitialized)
    {
        InitializeUnifiedGrid();
    }
    
    // Mark all cells as safe initially
    for (auto& Pair : UnifiedGrid)
    {
        Pair.Value.bIsSafe = true;
    }
    
    // Calculate expanded scene bounds for safety
    FBox SceneBounds = ExpandedSceneBounds;
    FVector BoundsMin = SceneBounds.Min;
    FVector BoundsSize = SceneBounds.GetSize();
    
    int32 NumX = FMath::Max(1, FMath::CeilToInt(BoundsSize.X / GridResolution));
    int32 NumY = FMath::Max(1, FMath::CeilToInt(BoundsSize.Y / GridResolution));
    int32 NumZ = FMath::Max(1, FMath::CeilToInt(BoundsSize.Z / GridResolution));
    
    UE_LOG(LogTemp, Display, TEXT("Generating safe zone grid: %dx%dx%d "
                                  "with cell size %.2f"), NumX, NumY, NumZ, GridResolution);
    
    // OPTIMIZATION: Use parallel processing for large meshes
    const bool bUseParallelProcessing = SceneMeshes.Num() > 10 || TotalTrianglesInScene > 100000;
    FCriticalSection CriticalSection;
    
    // For each mesh in the scene
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        // Process triangles in parallel for large meshes
        if (bUseParallelProcessing && MeshInfo.Indices.Num() > 30000)
        {
            ParallelFor(MeshInfo.Indices.Num() / 3, [&](int32 TriangleIndex)
            {
                int32 i = TriangleIndex * 3;
                if (i + 2 >= MeshInfo.Indices.Num()) return;
                
                // Get triangle vertices
                const FVector& V0 = MeshInfo.VertexPositions[MeshInfo.Indices[i]];
                const FVector& V1 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 1]];
                const FVector& V2 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 2]];
                
                // Calculate triangle bounds
                FBox TriBounds(EForceInit::ForceInit);
                TriBounds += V0;
                TriBounds += V1;
                TriBounds += V2;
                
                // Expand bounds by safe distance
                TriBounds = TriBounds.ExpandBy(FVector(SafeDistance,
                    SafeDistance, SafeHeight));
                
                // Convert to grid coordinates
                int32 MinX = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.X - BoundsMin.X) / GridResolution));
                int32 MinY = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.Y - BoundsMin.Y) / GridResolution));
                int32 MinZ = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.Z - BoundsMin.Z) / GridResolution));
                
                int32 MaxX = FMath::Min(NumX - 1, FMath::CeilToInt(
                    (TriBounds.Max.X - BoundsMin.X) / GridResolution));
                int32 MaxY = FMath::Min(NumY - 1, FMath::CeilToInt(
                    (TriBounds.Max.Y - BoundsMin.Y) / GridResolution));
                int32 MaxZ = FMath::Min(NumZ - 1, FMath::CeilToInt(
                    (TriBounds.Max.Z - BoundsMin.Z) / GridResolution));
                
                // Process cells that need checking
                TArray<FIntVector> CellsToUpdate;
                
                // Check each cell in the expanded bounds
                for (int32 X = MinX; X <= MaxX; ++X)
                {
                    for (int32 Y = MinY; Y <= MaxY; ++Y)
                    {
                        for (int32 Z = MinZ; Z <= MaxZ; ++Z)
                        {
                            FIntVector GridCoords(X, Y, Z);
                            
                            // Calculate cell center
                            FVector CellCenter(
                                BoundsMin.X + (X + 0.5f) * GridResolution,
                                BoundsMin.Y + (Y + 0.5f) * GridResolution,
                                BoundsMin.Z + (Z + 0.5f) * GridResolution
                            );
                            
                            // Find closest point on triangle
                            FVector ClosestPoint = FMath::ClosestPointOnTriangleToPoint(
                                CellCenter, V0, V1, V2);
                            
                            // Check horizontal distance (XY plane)
                            FVector2D CellCenter2D(CellCenter.X, CellCenter.Y);
                            FVector2D ClosestPoint2D(ClosestPoint.X, ClosestPoint.Y);
                            float HorizontalDist = FVector2D::Distance(CellCenter2D, ClosestPoint2D);
                            
                            // Check vertical distance (Z axis)
                            float VerticalDist = FMath::Abs(CellCenter.Z - ClosestPoint.Z);
                            
                            // Mark as unsafe if too close in both horizontal and vertical directions
                            if (HorizontalDist < SafeDistance && VerticalDist < SafeHeight)
                            {
                                CellsToUpdate.Add(GridCoords);
                            }
                        }
                    }
                }
                
                // Update grid cells in a thread-safe manner
                if (CellsToUpdate.Num() > 0)
                {
                    FScopeLock Lock(&CriticalSection);
                    for (const FIntVector& Coords : CellsToUpdate)
                    {
                        FUnifiedGridCell& Cell = UnifiedGrid.FindOrAdd(Coords);
                        Cell.bIsSafe = false;
                    }
                }
            });
        }
        else
        {
            // Sequential processing for smaller meshes
            for (int32 i = 0; i < MeshInfo.Indices.Num(); i += 3)
            {
                if (i + 2 >= MeshInfo.Indices.Num()) continue;
                
                // Get triangle vertices
                const FVector& V0 = MeshInfo.VertexPositions[MeshInfo.Indices[i]];
                const FVector& V1 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 1]];
                const FVector& V2 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 2]];
                
                // Calculate triangle bounds
                FBox TriBounds(EForceInit::ForceInit);
                TriBounds += V0;
                TriBounds += V1;
                TriBounds += V2;
                
                // Expand bounds by safe distance
                TriBounds = TriBounds.ExpandBy(FVector(SafeDistance,
                    SafeDistance, SafeHeight));
                
                // Convert to grid coordinates
                int32 MinX = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.X - BoundsMin.X) / GridResolution));
                int32 MinY = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.Y - BoundsMin.Y) / GridResolution));
                int32 MinZ = FMath::Max(0, FMath::FloorToInt(
                    (TriBounds.Min.Z - BoundsMin.Z) / GridResolution));
                
                int32 MaxX = FMath::Min(NumX - 1, FMath::CeilToInt(
                    (TriBounds.Max.X - BoundsMin.X) / GridResolution));
                int32 MaxY = FMath::Min(NumY - 1, FMath::CeilToInt(
                    (TriBounds.Max.Y - BoundsMin.Y) / GridResolution));
                int32 MaxZ = FMath::Min(NumZ - 1, FMath::CeilToInt(
                    (TriBounds.Max.Z - BoundsMin.Z) / GridResolution));
                
                // OPTIMIZATION: Use bounding box check first
                for (int32 X = MinX; X <= MaxX; ++X)
                {
                    for (int32 Y = MinY; Y <= MaxY; ++Y)
                    {
                        for (int32 Z = MinZ; Z <= MaxZ; ++Z)
                        {
                            FIntVector GridCoords(X, Y, Z);
                            
                            // Find or add cell to unified grid
                            FUnifiedGridCell& Cell = UnifiedGrid.FindOrAdd(GridCoords);
                            
                            // Skip if already marked unsafe (optimization)
                            if (!Cell.bIsSafe) continue;
                            
                            // Calculate cell center
                            FVector CellCenter(
                                BoundsMin.X + (X + 0.5f) * GridResolution,
                                BoundsMin.Y + (Y + 0.5f) * GridResolution,
                                BoundsMin.Z + (Z + 0.5f) * GridResolution
                            );
                            
                            // Find closest point on triangle
                            FVector ClosestPoint = FMath::ClosestPointOnTriangleToPoint(
                                CellCenter, V0, V1, V2);
                            
                            // Check horizontal distance (XY plane)
                            FVector2D CellCenter2D(CellCenter.X, CellCenter.Y);
                            FVector2D ClosestPoint2D(ClosestPoint.X, ClosestPoint.Y);
                            float HorizontalDist = FVector2D::Distance(CellCenter2D, ClosestPoint2D);
                            
                            // Check vertical distance (Z axis)
                            float VerticalDist = FMath::Abs(CellCenter.Z - ClosestPoint.Z);
                            
                            // Mark as unsafe if too close in both horizontal and vertical directions
                            if (HorizontalDist < SafeDistance && VerticalDist < SafeHeight)
                            {
                                Cell.bIsSafe = false;
                            }
                        }
                    }
                }
            }
        }
    }

    bSafeZoneDirty = true;
}

void ASceneAnalysisManager::InitializeSafeZoneVisualization()
{
    if (!World)
        return;
    
    // Initialize the procedural mesh component if it doesn't exist
    if (!SafeZoneVisualizationMesh)
    {
        SafeZoneVisualizationMesh = NewObject<UProceduralMeshComponent>(this);
        SafeZoneVisualizationMesh->RegisterComponent();
        SafeZoneVisualizationMesh->SetMobility(EComponentMobility::Movable);
        SafeZoneVisualizationMesh->AttachToComponent(GetRootComponent(), 
                                                  FAttachmentTransformRules::KeepWorldTransform);
        SafeZoneVisualizationMesh->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
    }
    
    // Load or create the safe zone material if not already set
    if (!SafeZoneMaterial)
    {
        // Try to load the safe zone material
        SafeZoneMaterial = LoadObject<UMaterialInterface>(nullptr, 
            TEXT("/Script/Engine.Material'/VCCSim/Materials/M_SafeZone.M_SafeZone'"));
        
        if (!SafeZoneMaterial)
        {
            UE_LOG(LogTemp, Error, TEXT("InitializeSafeZoneVisualization: "
                                       "Failed to load safe zone material."));
        }
    }
}

void ASceneAnalysisManager::VisualizeSafeZone(bool Vis)
{
    if (!World)
        return;
    
    // Check if we have a valid grid
    if (!bGridInitialized)
    {
        UE_LOG(LogTemp, Warning, TEXT("VisualizeSafeZone: No grid initialized"));
        return;
    }
    
    // If not showing, clear visualization and return
    if (!Vis)
    {
        if (SafeZoneVisualizationMesh)
        {
            SafeZoneVisualizationMesh->SetVisibility(false);
        }
        return;
    }
    
    // Initialize visualization components if needed
    InitializeSafeZoneVisualization();
    
    // If mesh component failed to initialize, return
    if (!SafeZoneVisualizationMesh)
    {
        UE_LOG(LogTemp, Error, TEXT("VisualizeSafeZone: Safe zone "
                                   "mesh component not initialized"));
        return;
    }
    
    // Only update mesh if dirty or visibility is changing
    if (bSafeZoneDirty)
    {
        CreateSafeZoneMesh();
        bSafeZoneDirty = false;
    }
    
    // Set visibility
    SafeZoneVisualizationMesh->SetVisibility(true);
}

void ASceneAnalysisManager::ClearSafeZoneVisualization()
{
    if (SafeZoneVisualizationMesh)
    {
        SafeZoneVisualizationMesh->ClearAllMeshSections();
        SafeZoneVisualizationMesh->SetVisibility(false);
    }
}

bool ASceneAnalysisManager::IsPointVisibleFromCamera(
    const FVector& Point, const FTransform& CameraPose) const
{
    if (!World)
    {
        UE_LOG(LogTemp, Warning, TEXT("IsPointVisibleFromCamera: No valid world"));
        return false;
    }
    
    // Get camera position
    FVector CameraPos = CameraPose.GetLocation();
    
    // Check if point is in front of camera (dot product optimization)
    FVector CameraForward = CameraPose.GetRotation().GetForwardVector();
    FVector PointDir = (Point - CameraPos).GetSafeNormal();
    
    if (FVector::DotProduct(CameraForward, PointDir) <= 0.0f)
        return false; // Point is behind camera
    
    // Trace from camera to point
    FHitResult HitResult;
    FCollisionQueryParams TraceParams;
    TraceParams.bTraceComplex = true;
    
    if (World->LineTraceSingleByChannel(
            HitResult,
            CameraPos,
            Point,
            ECC_Visibility,
            TraceParams))
    {
        // If we hit something near our target point, consider it visible
        return (HitResult.Location - Point).SizeSquared() < 1.0f;
    }
    
    // If nothing was hit, point is visible
    return true;
}

TArray<FVector> ASceneAnalysisManager::SamplePointsOnMesh(const FMeshInfo& MeshInfo)
{
    TArray<FVector> SampledPoints;
    
    if (bUseVertexSampling)
    {
        // Use vertices directly for efficiency
        return MeshInfo.VertexPositions;
    }
    
    // Pre-allocate for efficiency (rough estimation)
    SampledPoints.Reserve(MeshInfo.NumTriangles * 5); // 3 vertices plus estimated samples
    
    // Sample points based on triangles
    for (int32 i = 0; i < MeshInfo.Indices.Num(); i += 3)
    {
        if (i + 2 < MeshInfo.Indices.Num())
        {
            const FVector& V0 = MeshInfo.VertexPositions[MeshInfo.Indices[i]];
            const FVector& V1 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 1]];
            const FVector& V2 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 2]];
            
            // Add triangle vertices
            SampledPoints.Add(V0);
            SampledPoints.Add(V1);
            SampledPoints.Add(V2);
            
            // Calculate triangle area (in square units)
            float TriangleArea = 0.5f * FVector::CrossProduct(V1 - V0, V2 - V0).Size();
            
            // Calculate number of samples based on SamplingDensity and triangle area
            // SamplingDensity represents points per square meter
            int32 NumSamples = FMath::Max(1, FMath::RoundToInt(TriangleArea * SamplingDensity));
            
            // Add additional samples within the triangle
            for (int32 SampleIdx = 0; SampleIdx < NumSamples; ++SampleIdx)
            {
                // Generate random barycentric coordinates
                float r1 = FMath::SRand();
                float r2 = FMath::SRand();
                
                // Convert to barycentric coordinates
                float u = 1.0f - FMath::Sqrt(r1);
                float v = r2 * FMath::Sqrt(r1);
                float w = 1.0f - u - v;
                
                // Compute point using barycentric coordinates
                FVector SamplePoint = u * V0 + v * V1 + w * V2;
                SampledPoints.Add(SamplePoint);
            }
        }
    }
    UE_LOG(LogTemp, Warning, TEXT("Sampled %d points from mesh %s"),
        SampledPoints.Num(), *MeshInfo.MeshName);
    
    return SampledPoints;
}

void ASceneAnalysisManager::UpdateCoverageGrid()
{
    if (!bGridInitialized)
        return;
    
    // Reset visible points and coverage in existing cells
    for (auto& Pair : UnifiedGrid)
    {
        FUnifiedGridCell& Cell = Pair.Value;
        Cell.VisiblePoints = 0;
        Cell.Coverage = 0.0f;
        // Note: we don't reset safe zone data here
    }
    
    // Process each point in the coverage map
    for (const auto& Pair : CoverageMap)
    {
        const FVector& Point = Pair.Key;
        bool bIsVisible = Pair.Value;
        
        // Convert world position to grid coordinates
        FIntVector GridCoords = WorldToGridCoordinates(Point);
        
        // Get the cell (should already exist from initialization)
        if (UnifiedGrid.Contains(GridCoords))
        {
            FUnifiedGridCell& Cell = UnifiedGrid[GridCoords];
            
            // If point is visible, increment visible points
            if (bIsVisible)
            {
                Cell.VisiblePoints++;
            }
        }
    }
    
    // Normalize coverage values
    for (auto& Pair : UnifiedGrid)
    {
        FUnifiedGridCell& Cell = Pair.Value;
        if (Cell.TotalPoints > 0)
        {
            Cell.Coverage = (float)Cell.VisiblePoints / (float)Cell.TotalPoints;
        }
    }
}

void ASceneAnalysisManager::CreateCoverageMesh()
{
    // Clear existing mesh sections
    CoverageVisualizationMesh->ClearAllMeshSections();
    
    // Create arrays for procedural mesh
    TArray<FVector> Vertices;
    TArray<int32> Triangles;
    TArray<FVector> Normals;
    TArray<FVector2D> UV0;
    TArray<FColor> VertexColors;
    TArray<FProcMeshTangent> Tangents;
    
    // Pre-allocate memory based on number of cells with sample points
    int32 EstimatedCells = UnifiedGrid.Num();
    Vertices.Reserve(EstimatedCells * 8);
    Triangles.Reserve(EstimatedCells * 36);
    Normals.Reserve(EstimatedCells * 8);
    UV0.Reserve(EstimatedCells * 8);
    VertexColors.Reserve(EstimatedCells * 8);
    Tangents.Reserve(EstimatedCells * 8);
    
    // Create cubes only for cells with sample points and coverage above threshold
    int32 NumCellsVisualized = 0;
    for (const auto& Pair : UnifiedGrid)
    {
        const FIntVector& GridCoords = Pair.Key;
        const FUnifiedGridCell& Cell = Pair.Value;
        
        // Skip cells with no sample points or low coverage
        if (Cell.TotalPoints == 0 || Cell.Coverage < VisualizationThreshold)
            continue;
        
        // Calculate cell center in world space
        FVector CellCenter(
            GridOrigin.X + (GridCoords.X + 0.5f) * GridResolution,
            GridOrigin.Y + (GridCoords.Y + 0.5f) * GridResolution,
            GridOrigin.Z + (GridCoords.Z + 0.5f) * GridResolution
        );
        
        // Calculate cell size (slightly smaller than grid resolution to see cell boundaries)
        float CellSize = GridResolution * 0.9f;
        float HalfSize = CellSize * 0.5f;
        
        // Convert coverage to color (green = 1.0, red = 0.0)
        FColor CellColor = FLinearColor::LerpUsingHSV(
            FLinearColor(1.0f, 0.0f, 0.0f), // Red (0% coverage)
            FLinearColor(0.0f, 1.0f, 0.0f), // Green (100% coverage)
            Cell.Coverage
        ).ToFColor(false);
        
        // Adjust alpha based on coverage
        CellColor.A = 128 + FMath::FloorToInt(127.0f * Cell.Coverage); // 128-255 range for alpha
        
        // Add a cube for this cell
        int32 BaseVertexIndex = Vertices.Num();
        
        // Define the 8 corners of the cube
        Vertices.Add(CellCenter + FVector(-HalfSize, -HalfSize, -HalfSize)); // 0: bottom left back
        Vertices.Add(CellCenter + FVector(HalfSize, -HalfSize, -HalfSize));  // 1: bottom right back
        Vertices.Add(CellCenter + FVector(HalfSize, HalfSize, -HalfSize));   // 2: bottom right front
        Vertices.Add(CellCenter + FVector(-HalfSize, HalfSize, -HalfSize));  // 3: bottom left front
        Vertices.Add(CellCenter + FVector(-HalfSize, -HalfSize, HalfSize));  // 4: top left back
        Vertices.Add(CellCenter + FVector(HalfSize, -HalfSize, HalfSize));   // 5: top right back
        Vertices.Add(CellCenter + FVector(HalfSize, HalfSize, HalfSize));    // 6: top right front
        Vertices.Add(CellCenter + FVector(-HalfSize, HalfSize, HalfSize));   // 7: top left front
        
        // Add colors for all 8 vertices
        for (int32 i = 0; i < 8; ++i)
        {
            VertexColors.Add(CellColor);
        }
        
        // Add texture coordinates
        UV0.Add(FVector2D(0, 0)); // 0
        UV0.Add(FVector2D(1, 0)); // 1
        UV0.Add(FVector2D(1, 1)); // 2
        UV0.Add(FVector2D(0, 1)); // 3
        UV0.Add(FVector2D(0, 0)); // 4
        UV0.Add(FVector2D(1, 0)); // 5
        UV0.Add(FVector2D(1, 1)); // 6
        UV0.Add(FVector2D(0, 1)); // 7
        
        // Add normals
        Normals.Add(FVector(-1, -1, -1).GetSafeNormal()); // 0
        Normals.Add(FVector(1, -1, -1).GetSafeNormal());  // 1
        Normals.Add(FVector(1, 1, -1).GetSafeNormal());   // 2
        Normals.Add(FVector(-1, 1, -1).GetSafeNormal());  // 3
        Normals.Add(FVector(-1, -1, 1).GetSafeNormal());  // 4
        Normals.Add(FVector(1, -1, 1).GetSafeNormal());   // 5
        Normals.Add(FVector(1, 1, 1).GetSafeNormal());    // 6
        Normals.Add(FVector(-1, 1, 1).GetSafeNormal());   // 7
        
        // Add tangents (simplified)
        for (int32 i = 0; i < 8; ++i)
        {
            Tangents.Add(FProcMeshTangent(1, 0, 0));
        }
        
        // Add triangles for each face (12 triangles total)
        // Bottom face (0,1,2,3)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 3);
        
        // Top face (4,5,6,7)
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 6);
        
        // Front face (3,2,6,7)
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 7);
        
        // Back face (0,1,5,4)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 5);
        
        // Left face (0,3,7,4)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 4);
        
        // Right face (1,2,6,5)
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 6);
        
        NumCellsVisualized++;
    }
    
    // Only create mesh if we have cells to visualize
    if (NumCellsVisualized > 0)
    {
        // Create the mesh section
        CoverageVisualizationMesh->CreateMeshSection(0, Vertices, Triangles, Normals,
            UV0, VertexColors, Tangents, false);
        
        // Set the material
        if (CoverageMaterial)
        {
            CoverageVisualizationMesh->SetMaterial(0, CoverageMaterial);
        }
    }
    
    UE_LOG(LogTemp, Display, TEXT("Coverage visualization: Created mesh with %d cells "
                                  "visible out of %d populated cells (%d vertices, %d triangles)"), 
           NumCellsVisualized, UnifiedGrid.Num(), Vertices.Num(), Triangles.Num() / 3);
}

void ASceneAnalysisManager::CreateSafeZoneMesh()
{
    // Clear existing mesh sections
    SafeZoneVisualizationMesh->ClearAllMeshSections();
    
    // Create arrays for procedural mesh
    TArray<FVector> Vertices;
    TArray<int32> Triangles;
    TArray<FVector> Normals;
    TArray<FVector2D> UV0;
    TArray<FColor> VertexColors;
    TArray<FProcMeshTangent> Tangents;
    
    // Pre-allocate memory based on estimated number of unsafe cells
    int32 EstimatedUnsafeCells = UnifiedGrid.Num() / 2; // Rough estimate
    Vertices.Reserve(EstimatedUnsafeCells * 8);
    Triangles.Reserve(EstimatedUnsafeCells * 36);
    Normals.Reserve(EstimatedUnsafeCells * 8);
    UV0.Reserve(EstimatedUnsafeCells * 8);
    VertexColors.Reserve(EstimatedUnsafeCells * 8);
    Tangents.Reserve(EstimatedUnsafeCells * 8);
    
    // Create cubes only for unsafe cells
    int32 NumCellsVisualized = 0;
    for (const auto& Pair : UnifiedGrid)
    {
        const FIntVector& GridCoords = Pair.Key;
        const FUnifiedGridCell& Cell = Pair.Value;
        
        // Skip safe cells - only visualize unsafe cells
        if (Cell.bIsSafe)
            continue;
        
        // Calculate cell center in world space
        FVector CellCenter(
            GridOrigin.X + (GridCoords.X + 0.5f) * GridResolution,
            GridOrigin.Y + (GridCoords.Y + 0.5f) * GridResolution,
            GridOrigin.Z + (GridCoords.Z + 0.5f) * GridResolution
        );
        
        // Calculate cell size (slightly smaller than grid resolution to see cell boundaries)
        float CellSize = GridResolution * 0.9f;
        float HalfSize = CellSize * 0.5f;
        
        // Use a consistent color for unsafe cells
        FColor CellColor = FColor(255, 0, 0, 128); // Red with transparency
        
        // Add a cube for this cell
        int32 BaseVertexIndex = Vertices.Num();
        
        // Define the 8 corners of the cube
        Vertices.Add(CellCenter + FVector(-HalfSize, -HalfSize, -HalfSize)); // 0: bottom left back
        Vertices.Add(CellCenter + FVector(HalfSize, -HalfSize, -HalfSize));  // 1: bottom right back
        Vertices.Add(CellCenter + FVector(HalfSize, HalfSize, -HalfSize));   // 2: bottom right front
        Vertices.Add(CellCenter + FVector(-HalfSize, HalfSize, -HalfSize));  // 3: bottom left front
        Vertices.Add(CellCenter + FVector(-HalfSize, -HalfSize, HalfSize));  // 4: top left back
        Vertices.Add(CellCenter + FVector(HalfSize, -HalfSize, HalfSize));   // 5: top right back
        Vertices.Add(CellCenter + FVector(HalfSize, HalfSize, HalfSize));    // 6: top right front
        Vertices.Add(CellCenter + FVector(-HalfSize, HalfSize, HalfSize));   // 7: top left front
        
        // Add colors for all 8 vertices
        for (int32 i = 0; i < 8; ++i)
        {
            VertexColors.Add(CellColor);
        }
        
        // Add texture coordinates
        UV0.Add(FVector2D(0, 0)); // 0
        UV0.Add(FVector2D(1, 0)); // 1
        UV0.Add(FVector2D(1, 1)); // 2
        UV0.Add(FVector2D(0, 1)); // 3
        UV0.Add(FVector2D(0, 0)); // 4
        UV0.Add(FVector2D(1, 0)); // 5
        UV0.Add(FVector2D(1, 1)); // 6
        UV0.Add(FVector2D(0, 1)); // 7
        
        // Add normals
        Normals.Add(FVector(-1, -1, -1).GetSafeNormal()); // 0
        Normals.Add(FVector(1, -1, -1).GetSafeNormal());  // 1
        Normals.Add(FVector(1, 1, -1).GetSafeNormal());   // 2
        Normals.Add(FVector(-1, 1, -1).GetSafeNormal());  // 3
        Normals.Add(FVector(-1, -1, 1).GetSafeNormal());  // 4
        Normals.Add(FVector(1, -1, 1).GetSafeNormal());   // 5
        Normals.Add(FVector(1, 1, 1).GetSafeNormal());    // 6
        Normals.Add(FVector(-1, 1, 1).GetSafeNormal());   // 7
        
        // Add tangents (simplified)
        for (int32 i = 0; i < 8; ++i)
        {
            Tangents.Add(FProcMeshTangent(1, 0, 0));
        }
        
        // Add triangles for each face (12 triangles total)
        // Bottom face (0,1,2,3)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 3);
        
        // Top face (4,5,6,7)
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 6);
        
        // Front face (3,2,6,7)
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 7);
        
        // Back face (0,1,5,4)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 4);
        Triangles.Add(BaseVertexIndex + 5);
        
        // Left face (0,3,7,4)
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 3);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 0);
        Triangles.Add(BaseVertexIndex + 7);
        Triangles.Add(BaseVertexIndex + 4);
        
        // Right face (1,2,6,5)
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 6);
        Triangles.Add(BaseVertexIndex + 2);
        Triangles.Add(BaseVertexIndex + 1);
        Triangles.Add(BaseVertexIndex + 5);
        Triangles.Add(BaseVertexIndex + 6);
        
        NumCellsVisualized++;
    }
    
    // Only create mesh if we have cells to visualize
    if (NumCellsVisualized > 0)
    {
        // Create the mesh section
        SafeZoneVisualizationMesh->CreateMeshSection(0, Vertices, Triangles, Normals,
            UV0, VertexColors, Tangents, false);
        
        // Set the material
        if (SafeZoneMaterial)
        {
            SafeZoneVisualizationMesh->SetMaterial(0, SafeZoneMaterial);
        }
    }
    
    UE_LOG(LogTemp, Display, TEXT("Safe zone visualization: Created mesh with %d unsafe cells "
                                  "out of %d populated cells (%d vertices, %d triangles)"), 
           NumCellsVisualized, UnifiedGrid.Num(), Vertices.Num(), Triangles.Num() / 3);
}

/* ----------------------------- Test ----------------------------- */

void ASceneAnalysisManager::ExportMeshesToPly()
{
    if (!World)
    {
        UE_LOG(LogTemp, Warning, TEXT("USceneAnalysisManager::ExportMeshesToPly:"
            " World not set!"));
        return;
    }

    if (SceneMeshes.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("USceneAnalysisManager::ExportMeshesToPly:"
            " No meshes found in the scene!"));
        return;
    }
    
    // Create export directory if it doesn't exist
    IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
    if (!PlatformFile.DirectoryExists(*LogPath))
    {
        PlatformFile.CreateDirectory(*LogPath);
    }

    
    // Export each mesh to a separate PLY file
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        FString FilePath = FPaths::Combine(LogPath, FString::Printf(TEXT("%s_%d.ply"),
            *MeshInfo.MeshName, MeshInfo.MeshID));
        FFileHelper::SaveStringToFile(GeneratePlyContent(MeshInfo), *FilePath);
    }
}

FString ASceneAnalysisManager::GeneratePlyContent(const FMeshInfo& MeshInfo)
{
    FString PLYContent;
    
    // PLY Header
    PLYContent += TEXT("ply\n");
    PLYContent += TEXT("format ascii 1.0\n");
    PLYContent += FString::Printf(TEXT("element vertex %d\n"), MeshInfo.VertexPositions.Num());
    PLYContent += TEXT("property float x\n");
    PLYContent += TEXT("property float y\n");
    PLYContent += TEXT("property float z\n");
    PLYContent += FString::Printf(TEXT("element face %d\n"), MeshInfo.NumTriangles);
    PLYContent += TEXT("property list uchar int vertex_indices\n");
    PLYContent += TEXT("end_header\n");
    
    // Vertex data
    for (const FVector& Vertex : MeshInfo.VertexPositions)
    {
        PLYContent += FString::Printf(TEXT("%f %f %f\n"), Vertex.X, Vertex.Y, Vertex.Z);
    }
    
    // Face data
    for (int32 i = 0; i < MeshInfo.Indices.Num(); i += 3)
    {
        if (i + 2 < MeshInfo.Indices.Num())
        {
            PLYContent += FString::Printf(TEXT("3 %d %d %d\n"), 
                MeshInfo.Indices[i], 
                MeshInfo.Indices[i + 1], 
                MeshInfo.Indices[i + 2]);
        }
    }
    
    return PLYContent;
}

void ASceneAnalysisManager::VisualizeSceneMeshes(
    float Duration, bool bShowWireframe, bool bShowVertices, float VertexSize)
{
    if (!World || SceneMeshes.Num() == 0)
        return;
    
    // Generate a unique color for each mesh for easier distinction
    TArray<FColor> MeshColors;
    for (int32 i = 0; i < SceneMeshes.Num(); ++i)
    {
        // Create visually distinct colors using golden ratio
        const float Hue = fmodf(i * 0.618033988749895f, 1.0f);
        FLinearColor LinearColor = FLinearColor::MakeFromHSV8(Hue * 255.0f, 200, 200);
        MeshColors.Add(LinearColor.ToFColor(false));
    }
    
    // Visualize each mesh
    for (int32 MeshIdx = 0; MeshIdx < SceneMeshes.Num(); ++MeshIdx)
    {
        const FMeshInfo& MeshInfo = SceneMeshes[MeshIdx];
        const FColor& Color = MeshColors[MeshIdx];
        
        // Draw mesh bounds
        DrawDebugBox(World, MeshInfo.Bounds.Origin, MeshInfo.Bounds.BoxExtent,
            Color, false, Duration, 0, 2.0f);
        
        // Draw mesh ID text
        FString MeshText = FString::Printf(TEXT("Mesh ID: %d\nName: %s\nTriangles: %d"), 
            MeshInfo.MeshID, *MeshInfo.MeshName, MeshInfo.NumTriangles);
        DrawDebugString(World, MeshInfo.Bounds.Origin, MeshText, nullptr, Color, Duration);
        
        // Draw wireframe if requested
        if (bShowWireframe)
        {
            for (int32 i = 0; i < MeshInfo.Indices.Num(); i += 3)
            {
                if (i + 2 < MeshInfo.Indices.Num())
                {
                    const FVector& V0 = MeshInfo.VertexPositions[MeshInfo.Indices[i]];
                    const FVector& V1 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 1]];
                    const FVector& V2 = MeshInfo.VertexPositions[MeshInfo.Indices[i + 2]];
                    
                    DrawDebugLine(World, V0, V1, Color, false,
                        Duration, 0, 1.0f);
                    DrawDebugLine(World, V1, V2, Color, false,
                        Duration, 0, 1.0f);
                    DrawDebugLine(World, V2, V0, Color, false,
                        Duration, 0, 1.0f);
                }
            }
        }
        
        // Draw vertices if requested
        if (bShowVertices)
        {
            for (const FVector& Vertex : MeshInfo.VertexPositions)
            {
                DrawDebugPoint(World, Vertex, VertexSize, Color,
                    false, Duration);
            }
        }
    }
    
    // Log some statistics
    UE_LOG(LogTemp, Display, TEXT("Visualized %d meshes with %d total "
                                  "triangles and %d total vertices"), 
        SceneMeshes.Num(), TotalTrianglesInScene, TotalPointsInScene);
}

void ASceneAnalysisManager::VisualizeSampledPoints(float Duration, float VertexSize)
{
    for (const auto& Point : CoverageMap)
    {
        DrawDebugPoint(World, Point.Key, VertexSize, FColor::Green,
            false, Duration);
    }
}
