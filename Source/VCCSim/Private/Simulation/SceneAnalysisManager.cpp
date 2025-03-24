#include "Simulation/SceneAnalysisManager.h"
#include "Components/StaticMeshComponent.h"
#include "Sensors/CameraSensor.h"
#include "Engine/StaticMesh.h"
#include "DrawDebugHelpers.h"
#include "EngineUtils.h"

USceneAnalysisManager::USceneAnalysisManager()
{
    World = nullptr;
    TotalPointsInScene = 0;
    TotalTrianglesInScene = 0;
    CurrentCoveragePercentage = 0.0f;
    SamplingDensity = 1.0f;
    bUseVertexSampling = false;
    GridResolution = 10.0f;
}

bool USceneAnalysisManager::Initialize(UWorld* InWorld)
{
    if (!InWorld)
        return false;
    
    World = InWorld;
    return true;
}

void USceneAnalysisManager::ScanScene()
{
    if (!World)
        return;
    
    // Clear previous data
    SceneMeshes.Empty();
    TotalPointsInScene = 0;
    TotalTrianglesInScene = 0;
    
    // Iterate through all static mesh components in the world
    for (TActorIterator<AActor> ActorItr(World); ActorItr; ++ActorItr)
    {
        TArray<UStaticMeshComponent*> MeshComponents;
        ActorItr->GetComponents<UStaticMeshComponent>(MeshComponents);
        
        for (UStaticMeshComponent* MeshComp : MeshComponents)
        {
            if (MeshComp && MeshComp->GetStaticMesh())
            {
                FMeshInfo MeshInfo;
                ExtractMeshData(MeshComp, MeshInfo);
                SceneMeshes.Add(MeshInfo);
                
                TotalTrianglesInScene += MeshInfo.NumTriangles;
                TotalPointsInScene += MeshInfo.NumVertices;
            }
        }
    }
    
    // Reset coverage data
    ResetCoverage();
}

void USceneAnalysisManager::RegisterCamera(URGBCameraComponent* CameraComponent)
{
    CameraIntrinsics.Add(CameraComponent->CameraName,
    CameraComponent->GetCameraIntrinsics());
    
    CameraComponent->OnKeyPointCaptured.BindUFunction(
        this, "UpdateAccumulatedCoverage");
}

FMeshInfo USceneAnalysisManager::GetMeshInfo(int32 MeshID) const
{
    for (const FMeshInfo& MeshInfo : SceneMeshes)
    {
        if (MeshInfo.MeshID == MeshID)
            return MeshInfo;
    }
    
    return FMeshInfo();
}

TArray<FMeshInfo> USceneAnalysisManager::GetAllMeshInfo() const
{
    return SceneMeshes;
}

void USceneAnalysisManager::UpdateAccumulatedCoverage(const FTransform& Transform, const FString& Name)
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

float USceneAnalysisManager::GetTotalCoveragePercentage() const
{
    return CurrentCoveragePercentage;
}

void USceneAnalysisManager::ResetCoverage()
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

void USceneAnalysisManager::VisualizeCoverage(bool bShowVisiblePoints, bool bHighlightCoveredMeshes, float Duration)
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
                DrawDebugPoint(World, PointPair.Key, 5.0f, FColor::Green, false, Duration);
            }
        }
    }
    
    // Highlight covered meshes
    if (bHighlightCoveredMeshes)
    {
        for (const FMeshInfo& MeshInfo : SceneMeshes)
        {
            FColor Color = CurrentlyVisibleMeshIDs.Contains(MeshInfo.MeshID) ? FColor::Green : FColor::Red;
            DrawDebugBox(World, MeshInfo.Bounds.Origin, MeshInfo.Bounds.BoxExtent, Color, false, Duration);
        }
    }
}

void USceneAnalysisManager::ConstructFrustum(FConvexVolume& OutFrustum, const FTransform& CameraPose, const FMatrix44f& CameraIntrinsic)
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
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(FarBottomLeft - Position, FarTopLeft - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(FarTopRight - Position, FarBottomRight - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(FarTopLeft - Position, FarTopRight - Position).GetSafeNormal()));
    OutFrustum.Planes.Add(FPlane(Position, FVector::CrossProduct(FarBottomRight - Position, FarBottomLeft - Position).GetSafeNormal()));
}

void USceneAnalysisManager::ExtractMeshData(UStaticMeshComponent* MeshComponent, FMeshInfo& OutMeshInfo)
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
            FVector3f VertexPos3f = LODModel.VertexBuffers.PositionVertexBuffer.VertexPosition(VertIdx);
            
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

bool USceneAnalysisManager::IsPointVisibleFromCamera(const FVector& Point, const FTransform& CameraPose) const
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

TArray<FVector> USceneAnalysisManager::SamplePointsOnMesh(const FMeshInfo& MeshInfo, int32 SamplesPerTriangle)
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