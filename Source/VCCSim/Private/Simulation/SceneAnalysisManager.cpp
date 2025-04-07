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
    SamplingDensity = 1.0f;
    bUseVertexSampling = false;
    LogPath = FPaths::ProjectLogDir();
    SafeZoneMaterial = nullptr;
    SafeZoneInstancedMesh = nullptr;

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
    // VisualizeSceneMeshes(10.0f, true, false, 0);
}

void ASceneAnalysisManager::RegisterCamera(URGBCameraComponent* CameraComponent)
{
    CameraIntrinsics.Add(CameraComponent->CameraName,
    CameraComponent->GetCameraIntrinsics());
    
    CameraComponent->OnKeyPointCaptured.BindUFunction(
        this, "UpdateAccumulatedCoverage");
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

void ASceneAnalysisManager::UpdateAccumulatedCoverage(
    const FTransform& Transform, const FString& Name)
{
    if (!World || SceneMeshes.Num() == 0)
        return;
    
    // Get camera intrinsics
    const FMatrix44f* CameraMatrix = CameraIntrinsics.Find(Name);
    if (!CameraMatrix)
        return;
    
    // Construct frustum for visibility checks
    FConvexVolume Frustum;
    ConstructFrustum(Frustum, Transform, *CameraMatrix);
    
    // Reset current visibility data
    CurrentlyVisibleMeshIDs.Empty();
    
    // Check visibility for each mesh
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        // Quick frustum-bounds check
        if (!Frustum.IntersectBox(MeshInfo.Bounds.Origin, MeshInfo.Bounds.BoxExtent))
            continue;
        
        // Sample points on the mesh
        TArray<FVector> SampledPoints = SamplePointsOnMesh(MeshInfo);
        
        // Check visibility for each point
        int32 VisiblePoints = 0;
        for (const FVector& Point : SampledPoints)
        {
            if (IsPointVisibleFromCamera(Point, Transform))
            {
                VisiblePoints++;
                CoverageMap.FindOrAdd(Point, true);
            }
        }
        
        // If any points are visible, mark mesh as visible
        if (VisiblePoints > 0)
        {
            CurrentlyVisibleMeshIDs.Add(MeshInfo.MeshID);
        }
    }
    
    // Update coverage percentage
    int32 CoveredPoints = 0;
    for (const auto& PointPair : CoverageMap)
    {
        if (PointPair.Value)
            CoveredPoints++;
    }
    
    CurrentCoveragePercentage = TotalPointsInScene > 0 ? 
        static_cast<float>(CoveredPoints) / TotalPointsInScene * 100.0f : 0.0f;
}

float ASceneAnalysisManager::GetTotalCoveragePercentage() const
{
    return CurrentCoveragePercentage;
}

void ASceneAnalysisManager::ResetCoverage()
{
    CoverageMap.Empty();
    CurrentlyVisibleMeshIDs.Empty();
    CurrentCoveragePercentage = 0.0f;
    
    // Initialize coverage map with all points set to not visible
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        TArray<FVector> SampledPoints = SamplePointsOnMesh(MeshInfo);
        for (const FVector& Point : SampledPoints)
        {
            CoverageMap.Add(Point, false);
        }
    }
}

void ASceneAnalysisManager::VisualizeCoverage(
    bool bShowVisiblePoints, bool bHighlightCoveredMeshes, float Duration)
{
    if (!World)
        return;
    
    // Visualize visible points
    if (bShowVisiblePoints)
    {
        for (const auto& PointPair : CoverageMap)
        {
            if (PointPair.Value) // If point is visible
            {
                DrawDebugPoint(World, PointPair.Key, 5.0f, FColor::Green,
                    false, Duration);
            }
        }
    }
    
    // Highlight covered meshes
    if (bHighlightCoveredMeshes)
    {
        for (const FMeshInfo& MeshInfo : SceneMeshes)
        {
            FColor Color = CurrentlyVisibleMeshIDs.Contains(MeshInfo.MeshID) ?
                FColor::Green : FColor::Red;
            DrawDebugBox(World, MeshInfo.Bounds.Origin,
                MeshInfo.Bounds.BoxExtent, Color, false, Duration);
        }
    }
}

void ASceneAnalysisManager::ConstructFrustum(
    FConvexVolume& OutFrustum, const FTransform& CameraPose, const FMatrix44f& CameraIntrinsic)
{
    // Get camera parameters
    float FOV = 2.0f * FMath::Atan(1.0f / CameraIntrinsic.M[0][0]); // FOV from focal length
    float AspectRatio = CameraIntrinsic.M[1][1] / CameraIntrinsic.M[0][0];
    
    // Create frustum planes
    const FVector ForwardVector = CameraPose.GetRotation().GetForwardVector();
    const FVector RightVector = CameraPose.GetRotation().GetRightVector();
    const FVector UpVector = CameraPose.GetRotation().GetUpVector();
    const FVector Position = CameraPose.GetLocation();
    
    // Near plane
    const float NearPlaneDistance = 10.0f;
    const float FarPlaneDistance = 10000.0f;
    
    // Calculate frustum corners
    const float HalfFOV = FOV * 0.5f;
    const float HalfFOVTan = FMath::Tan(HalfFOV);
    const float NearHeight = NearPlaneDistance * HalfFOVTan;
    const float NearWidth = NearHeight * AspectRatio;
    const float FarHeight = FarPlaneDistance * HalfFOVTan;
    const float FarWidth = FarHeight * AspectRatio;
    
    // Near and far plane centers
    const FVector NearCenter = Position + ForwardVector * NearPlaneDistance;
    const FVector FarCenter = Position + ForwardVector * FarPlaneDistance;
    
    // Near plane corners
    const FVector NearTopLeft = NearCenter + UpVector * NearHeight - RightVector * NearWidth;
    const FVector NearTopRight = NearCenter + UpVector * NearHeight + RightVector * NearWidth;
    const FVector NearBottomLeft = NearCenter - UpVector * NearHeight - RightVector * NearWidth;
    const FVector NearBottomRight = NearCenter - UpVector * NearHeight + RightVector * NearWidth;
    
    // Far plane corners
    const FVector FarTopLeft = FarCenter + UpVector * FarHeight - RightVector * FarWidth;
    const FVector FarTopRight = FarCenter + UpVector * FarHeight + RightVector * FarWidth;
    const FVector FarBottomLeft = FarCenter - UpVector * FarHeight - RightVector * FarWidth;
    const FVector FarBottomRight = FarCenter - UpVector * FarHeight + RightVector * FarWidth;
    
    // Create frustum planes
    OutFrustum.Planes.Empty(6);
    
    // Near plane
    OutFrustum.Planes.Add(FPlane(NearCenter, -ForwardVector));
    
    // Far plane
    OutFrustum.Planes.Add(FPlane(FarCenter, ForwardVector));
    
    // Side planes
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(
        FarBottomLeft - Position, FarTopLeft - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(
        FarTopRight - Position, FarBottomRight - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(
        FarTopLeft - Position, FarTopRight - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(
        FarBottomRight - Position, FarBottomLeft - Position).GetSafeNormal()));
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
    
    // Calculate scene bounds
    FBox SceneBounds(EForceInit::ForceInit);
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        SceneBounds += MeshInfo.Bounds.GetBox();
    }
    
    // Expand bounds to include safe zone
    SceneBounds = SceneBounds.ExpandBy(FVector(SafeDistance * 2.0f,
        SafeDistance * 2.0f, SafeHeight * 2.0f));
    
    // Calculate grid dimensions
    FVector BoundsSize = SceneBounds.GetSize();
    FVector BoundsMin = SceneBounds.Min;
    
    int32 NumX = FMath::Max(1, FMath::CeilToInt(BoundsSize.X / GridResolution));
    int32 NumY = FMath::Max(1, FMath::CeilToInt(BoundsSize.Y / GridResolution));
    int32 NumZ = FMath::Max(1, FMath::CeilToInt(BoundsSize.Z / GridResolution));
    
    UE_LOG(LogTemp, Display, TEXT("Generating safe zone grid: %dx%dx%d "
                                  "with cell size %.2f"), NumX, NumY, NumZ, GridResolution);
    
    // Create 3D boolean grid (true = safe, false = unsafe)
    SafeZoneGrid.SetNum(NumX);
    for (int32 X = 0; X < NumX; ++X)
    {
        SafeZoneGrid[X].SetNum(NumY);
        for (int32 Y = 0; Y < NumY; ++Y)
        {
            SafeZoneGrid[X][Y].Init(true, NumZ);
        }
    }
    
    // For each mesh in the scene
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        // Process each triangle
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
            
            // Check each cell in the expanded bounds
            for (int32 X = MinX; X <= MaxX; ++X)
            {
                for (int32 Y = MinY; Y <= MaxY; ++Y)
                {
                    for (int32 Z = MinZ; Z <= MaxZ; ++Z)
                    {
                        // Skip if already marked unsafe
                        if (!SafeZoneGrid[X][Y][Z]) continue;
                        
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
                            SafeZoneGrid[X][Y][Z] = false;
                        }
                    }
                }
            }
        }
    }
}

void ASceneAnalysisManager::VisualizeSafeZone(bool bPersistent)
{
    if (!World)
        return;
    
    // Check if we have a valid safe zone grid
    if (SafeZoneGrid.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("VisualizeSafeZone: No safe zone grid generated yet"));
        return;
    }
    
    // Create or clear the instanced mesh component
    if (!SafeZoneInstancedMesh)
    {
        // Create a new instanced mesh component
        SafeZoneInstancedMesh = NewObject<UInstancedStaticMeshComponent>(this);
        SafeZoneInstancedMesh->RegisterComponent();
        SafeZoneInstancedMesh->SetMobility(EComponentMobility::Movable);
        SafeZoneInstancedMesh->AttachToComponent(GetRootComponent(),
            FAttachmentTransformRules::KeepRelativeTransform);
        
        // Load the cube mesh
        UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr,
            TEXT("/Engine/BasicShapes/Cube.Cube"));
        
        if (CubeMesh)
        {
            SafeZoneInstancedMesh->SetStaticMesh(CubeMesh);
            
            // Disable Nanite for the cube mesh to support translucent materials
            if (CubeMesh->NaniteSettings.bEnabled)
            {
                UStaticMesh* MutableCubeMesh = const_cast<UStaticMesh*>(CubeMesh);
                if (MutableCubeMesh)
                {
                    MutableCubeMesh->NaniteSettings.bEnabled = false;
                    MutableCubeMesh->PostEditChange();
                }
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("VisualizeSafeZone: Failed to load cube mesh"));
            return;
        }
    }
    else
    {
        // Just clear existing instances
        SafeZoneInstancedMesh->ClearInstances();
    }
    
    // Set material if provided
    bool bIsTranslucent = false;
    
    if (SafeZoneMaterial)
    {
        // Check if the provided material is translucent
        EBlendMode BlendMode = SafeZoneMaterial->GetBlendMode();
        bIsTranslucent = (BlendMode == BLEND_Translucent);
        
        SafeZoneInstancedMesh->SetMaterial(0, SafeZoneMaterial);
    }
    else
    {
        // Create a dynamic material instance with a semi-transparent red color
        UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(
            UMaterial::GetDefaultMaterial(MD_Surface), this);
        
        if (DynamicMaterial)
        {
            // Set the blend mode to translucent
            DynamicMaterial->BlendMode = BLEND_Opaque;
            DynamicMaterial->SetScalarParameterValue(TEXT("Opacity"), 0.5f);
            DynamicMaterial->SetVectorParameterValue(TEXT("BaseColor"),
                FLinearColor(1.0f, 0.0f, 0.0f, 1.0f));
            
            SafeZoneInstancedMesh->SetMaterial(0, DynamicMaterial);
            bIsTranslucent = true;
        }
    }
    
    // If the material is translucent, make sure Nanite is disabled on the mesh
    if (bIsTranslucent && SafeZoneInstancedMesh->GetStaticMesh())
    {
        UStaticMesh* Mesh = const_cast<UStaticMesh*>(SafeZoneInstancedMesh->GetStaticMesh().Get());
        if (Mesh && Mesh->NaniteSettings.bEnabled)
        {
            Mesh->NaniteSettings.bEnabled = false;
            Mesh->PostEditChange();
            UE_LOG(LogTemp, Display, TEXT("Disabled Nanite on cube mesh "
                                          "to support translucent material"));
        }
    }
    
    // Get the scene bounds
    FBox SceneBounds(EForceInit::ForceInit);
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        SceneBounds += MeshInfo.Bounds.GetBox();
    }
    
    FVector BoundsMin = SceneBounds.Min;
    
    // Count how many unsafe cells we'll be visualizing
    int32 UnsafeCellCount = 0;
    
    // Add instances for each unsafe cell
    for (int32 X = 0; X < SafeZoneGrid.Num(); ++X)
    {
        for (int32 Y = 0; Y < SafeZoneGrid[X].Num(); ++Y)
        {
            for (int32 Z = 0; Z < SafeZoneGrid[X][Y].Num(); ++Z)
            {
                // Only visualize unsafe cells
                if (!SafeZoneGrid[X][Y][Z])
                {
                    // Calculate cell position
                    FVector CellPosition(
                        BoundsMin.X + (X + 0.5f) * GridResolution,
                        BoundsMin.Y + (Y + 0.5f) * GridResolution,
                        BoundsMin.Z + (Z + 0.5f) * GridResolution
                    );
                    
                    // Calculate transform
                    FTransform CellTransform(
                        FRotator::ZeroRotator,
                        CellPosition,
                        FVector(1.0f, 1.0f, 1.0f) * GridResolution / 100.0f
                        // Scale to match grid resolution (cube is 100 UE units)
                    );
                    
                    // Add instance
                    SafeZoneInstancedMesh->AddInstance(CellTransform);
                    UnsafeCellCount++;
                }
            }
        }
    }
    
    // If not persistent, set up a timer to clear the visualization
    if (!bPersistent && UnsafeCellCount > 0)
    {
        FTimerHandle TimerHandle;
        FTimerDelegate TimerDelegate;
        TimerDelegate.BindUObject(this, &ASceneAnalysisManager::ClearSafeZoneVisualization);
        
        World->GetTimerManager().SetTimer(TimerHandle, TimerDelegate, 5.0f, false);
    }
    
    UE_LOG(LogTemp, Display, TEXT("Visualized %d unsafe cells in the safe zone grid"),
        UnsafeCellCount);
}

void ASceneAnalysisManager::ClearSafeZoneVisualization()
{
    if (SafeZoneInstancedMesh)
    {
        SafeZoneInstancedMesh->ClearInstances();
    }
}

bool ASceneAnalysisManager::IsPointVisibleFromCamera(
    const FVector& Point, const FTransform& CameraPose) const
{
    if (!World)
        return false;
    
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

TArray<FVector> ASceneAnalysisManager::SamplePointsOnMesh(
    const FMeshInfo& MeshInfo, int32 SamplesPerTriangle)
{
    TArray<FVector> SampledPoints;
    
    if (bUseVertexSampling)
    {
        // Use vertices directly for efficiency
        return MeshInfo.VertexPositions;
    }
    
    // Pre-allocate for efficiency
    const int32 EstimatedPoints = MeshInfo.NumTriangles * (3 + SamplesPerTriangle);
    SampledPoints.Reserve(EstimatedPoints);
    
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
            
            // Add additional samples within the triangle
            for (int32 SampleIdx = 0; SampleIdx < SamplesPerTriangle; ++SampleIdx)
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
    
    return SampledPoints;
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