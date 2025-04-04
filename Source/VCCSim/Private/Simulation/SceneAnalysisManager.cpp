#include "Simulation/SceneAnalysisManager.h"
#include "Components/StaticMeshComponent.h"
#include "Sensors/CameraSensor.h"
#include "Engine/StaticMesh.h"
#include "Engine/StaticMeshActor.h"
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
    LogPath = FPaths::ProjectLogDir();
}

bool USceneAnalysisManager::Initialize(UWorld* InWorld, FString&& Path)
{
    if (!InWorld)
        return false;
    
    World = InWorld;
    LogPath = std::move(Path) + "/SceneAnalysisLog";
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

void USceneAnalysisManager::UpdateAccumulatedCoverage(
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

void USceneAnalysisManager::VisualizeCoverage(
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

void USceneAnalysisManager::ConstructFrustum(
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

void USceneAnalysisManager::ExtractMeshData(
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

bool USceneAnalysisManager::IsPointVisibleFromCamera(
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

TArray<FVector> USceneAnalysisManager::SamplePointsOnMesh(
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

void USceneAnalysisManager::ExportMeshesToPly()
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

FString USceneAnalysisManager::GeneratePlyContent(const FMeshInfo& MeshInfo)
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

void USceneAnalysisManager::VisualizeSceneMeshes(
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