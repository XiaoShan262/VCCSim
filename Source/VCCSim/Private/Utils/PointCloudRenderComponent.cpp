// #include "Utils/PointCloudRenderComponent.h"
// #include "DynamicMeshBuilder.h"
// #include "MaterialDomain.h"
// #include "PrimitiveSceneProxy.h"
// #include "RenderResource.h"
// #include "MeshBatch.h"
// #include "MaterialShared.h"
// #include "Engine/Engine.h"
//
// // GPU scene proxy for point cloud rendering
// class FPointCloudSceneProxy : public FPrimitiveSceneProxy
// {
// public:
//     FPointCloudSceneProxy(UPointCloudRenderComponent* Component)
//         : FPrimitiveSceneProxy(Component)
//     {
//         // Store component properties
//         const TArray<FPointData>& ComponentPoints = Component->Points;
//         Material = Component->PointMaterial;
//         
//         // Set material to default if not specified
//         if (!Material)
//         {
//             Material = UMaterial::GetDefaultMaterial(MD_Surface);
//         }
//         
//         // Store material relevance
//         MaterialRelevance = Material->GetRelevance(GetScene().GetFeatureLevel());
//         
//         // Set up the vertex buffer
//         VertexBufferPoints.Reserve(ComponentPoints.Num() * 4);
//         IndexBuffer.Reserve(ComponentPoints.Num() * 6);
//         
//         // Create a quad for each point (billboarded in the material)
//         uint32 VertexCount = 0;
//         
//         for (const FPointData& Point : ComponentPoints)
//         {
//             // Create a quad for the point
//             FDynamicMeshVertex V0, V1, V2, V3;
//             
//             // Set position for all vertices (will be billboarded in the material)
//             V0.Position = FVector3f(Point.Position);
//             V1.Position = FVector3f(Point.Position);
//             V2.Position = FVector3f(Point.Position);
//             V3.Position = FVector3f(Point.Position);
//             
//             // Set color
//             V0.Color = Point.Color;
//             V1.Color = Point.Color;
//             V2.Color = Point.Color;
//             V3.Color = Point.Color;
//             
//             // Set point size
//             float HalfSize = Point.Size * 0.5f;
//             
//             // Set UV coordinates (used for billboarding)
//             V0.TextureCoordinate[0] = FVector2f(-HalfSize, -HalfSize);
//             V1.TextureCoordinate[0] = FVector2f(HalfSize, -HalfSize);
//             V2.TextureCoordinate[0] = FVector2f(HalfSize, HalfSize);
//             V3.TextureCoordinate[0] = FVector2f(-HalfSize, HalfSize);
//             
//             // Add vertices
//             VertexBufferPoints.Add(V0);
//             VertexBufferPoints.Add(V1);
//             VertexBufferPoints.Add(V2);
//             VertexBufferPoints.Add(V3);
//             
//             // Add indices for two triangles
//             IndexBuffer.Add(VertexCount + 0);
//             IndexBuffer.Add(VertexCount + 1);
//             IndexBuffer.Add(VertexCount + 2);
//             
//             IndexBuffer.Add(VertexCount + 0);
//             IndexBuffer.Add(VertexCount + 2);
//             IndexBuffer.Add(VertexCount + 3);
//             
//             VertexCount += 4;
//         }
//     }
//     
//     virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, 
//                                        const FSceneViewFamily& ViewFamily,
//                                        uint32 VisibilityMap, 
//                                        FMeshElementCollector& Collector) const override
//     {
//         // Skip if no points
//         if (VertexBufferPoints.Num() == 0)
//             return;
//             
//         // Get material render proxy
//         FMaterialRenderProxy* MaterialProxy = Material->GetRenderProxy();
//             
//         // For each view
//         for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
//         {
//             if (VisibilityMap & (1 << ViewIndex))
//             {
//                 const FSceneView* View = Views[ViewIndex];
//                 
//                 // Create a new vertex factory for this view
//                 FLocalVertexFactory VertexFactory(ERHIFeatureLevel::SM5, "FPointCloudSceneProxy");
//                 
//                 // Allocate vertex and index buffers
//                 FStaticMeshVertexBuffers* VertexBuffers = &Collector.GetDynamicVertexBuffer();
//                 
//                 // Initialize vertex factory data
//                 FLocalVertexFactory::FDataType Data;
//                 
//                 // Set up vertex buffer
//                 TResourceArray<FDynamicMeshVertex> Vertices;
//                 Vertices.Append(VertexBufferPoints);
//                 
//                 // Set up position buffer
//                 VertexBuffers->PositionVertexBuffer.Init(Vertices.Num());
//                 for (int32 i = 0; i < Vertices.Num(); i++)
//                 {
//                     VertexBuffers->PositionVertexBuffer.VertexPosition(i) = Vertices[i].Position;
//                 }
//                 
//                 // Set up color buffer
//                 VertexBuffers->ColorVertexBuffer.Init(Vertices.Num());
//                 for (int32 i = 0; i < Vertices.Num(); i++)
//                 {
//                     VertexBuffers->ColorVertexBuffer.VertexColor(i) = Vertices[i].Color;
//                 }
//                 
//                 // Set up stat mesh buffer
//                 VertexBuffers->StaticMeshVertexBuffer.Init(Vertices.Num(), 1);
//                 for (int32 i = 0; i < Vertices.Num(); i++)
//                 {
//                     VertexBuffers->StaticMeshVertexBuffer.SetVertexTangents(
//                         i, 
//                         FVector3f(1, 0, 0), 
//                         FVector3f(0, 1, 0), 
//                         FVector3f(0, 0, 1)
//                     );
//                     VertexBuffers->StaticMeshVertexBuffer.SetVertexUV(
//                         i, 0, Vertices[i].TextureCoordinate[0]
//                     );
//                 }
//                 
//                 // Initialize vertex factory with data
//                 VertexBuffers->PositionVertexBuffer.InitResource();
//                 VertexBuffers->StaticMeshVertexBuffer.InitResource();
//                 VertexBuffers->ColorVertexBuffer.InitResource();
//                 
//                 Data.PositionComponentSRV = VertexBuffers->PositionVertexBuffer.GetSRV();
//                 Data.TangentsSRV = VertexBuffers->StaticMeshVertexBuffer.GetTangentsSRV();
//                 Data.TextureCoordinatesSRV = VertexBuffers->StaticMeshVertexBuffer.GetTexCoordsSRV();
//                 Data.ColorComponentsSRV = VertexBuffers->ColorVertexBuffer.GetColorSRV();
//                 
//                 VertexFactory.SetData(Data);
//                 VertexFactory.InitResource();
//                 Collector.AddVertexFactory(&VertexFactory);
//                 
//                 // Set up index buffer
//                 FRHIResourceCreateInfo CreateInfo(TEXT("PointCloudIndexBuffer"));
//                 FIndexBufferRHIRef IndexBufferRHI = RHICreateIndexBuffer(
//                     sizeof(uint32), 
//                     IndexBuffer.Num() * sizeof(uint32), 
//                     BUF_Static, 
//                     CreateInfo
//                 );
//                 
//                 void* IndexBufferData = RHILockBuffer(
//                     IndexBufferRHI, 
//                     0, 
//                     IndexBuffer.Num() * sizeof(uint32), 
//                     RLM_WriteOnly
//                 );
//                 FMemory::Memcpy(IndexBufferData, IndexBuffer.GetData(), IndexBuffer.Num() * sizeof(uint32));
//                 RHIUnlockBuffer(IndexBufferRHI);
//                 
//                 // Create mesh batch for rendering
//                 FMeshBatch& Mesh = Collector.AllocateMesh();
//                 FMeshBatchElement& BatchElement = Mesh.Elements[0];
//                 
//                 BatchElement.IndexBuffer = new(Collector.AllocateIndexBuffer(sizeof(uint32))) FIndexBuffer();
//                 ((FIndexBuffer*)BatchElement.IndexBuffer)->IndexBufferRHI = IndexBufferRHI;
//                 
//                 // Apply material                
//                 Mesh.MaterialRenderProxy = MaterialProxy;
//                 Mesh.VertexFactory = &VertexFactory;
//                 Mesh.LCI = nullptr;
//                 
//                 // Set primitive data
//                 FBoxSphereBounds LocalBounds = GetBounds();
//                 FMatrix WorldToLocal = GetLocalToWorld().Inverse();
//                 FBoxSphereBounds PreSkinnedLocalBounds = LocalBounds.TransformBy(WorldToLocal);
//                 
//                 BatchElement.PrimitiveUniformBuffer = CreatePrimitiveUniformBufferImmediate(
//                     GetLocalToWorld(), 
//                     GetBounds(), 
//                     LocalBounds, 
//                     PreSkinnedLocalBounds, 
//                     true, 
//                     false
//                 );
//                 
//                 BatchElement.FirstIndex = 0;
//                 BatchElement.NumPrimitives = IndexBuffer.Num() / 3;
//                 BatchElement.MinVertexIndex = 0;
//                 BatchElement.MaxVertexIndex = VertexBufferPoints.Num() - 1;
//                 
//                 Mesh.Type = PT_TriangleList;
//                 Mesh.bWireframe = false;
//                 Mesh.DepthPriorityGroup = SDPG_World;
//                 Mesh.bDisableBackfaceCulling = true;
//                 
//                 Collector.AddMesh(ViewIndex, Mesh);
//             }
//         }
//     }
//     
//     virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
//     {
//         FPrimitiveViewRelevance Result;
//         Result.bDrawRelevance = IsShown(View);
//         Result.bShadowRelevance = IsShadowCast(View);
//         Result.bDynamicRelevance = true;
//         MaterialRelevance.SetPrimitiveViewRelevance(Result);
//         return Result;
//     }
//     
//     virtual uint32 GetMemoryFootprint() const override
//     {
//         return sizeof(*this) + GetAllocatedSize();
//     }
//     
//     virtual SIZE_T GetTypeHash() const override
//     {
//         static size_t UniquePointer;
//         return reinterpret_cast<size_t>(&UniquePointer);
//     }
//     
// private:
//     UMaterialInterface* Material;
//     FMaterialRelevance MaterialRelevance;
//     TArray<FDynamicMeshVertex> VertexBufferPoints;
//     TArray<uint32> IndexBuffer;
// };
//
// UPointCloudRenderComponent::UPointCloudRenderComponent()
// {
//     PrimaryComponentTick.bCanEverTick = false;
//     SetCollisionEnabled(ECollisionEnabled::NoCollision);
//     
//     // Load default point material
//     static ConstructorHelpers::FObjectFinder<UMaterialInterface> DefaultMat(
//         TEXT("/Engine/BasicShapes/BasicShapeMaterial"));
//     if (DefaultMat.Succeeded())
//     {
//         PointMaterial = DefaultMat.Object;
//     }
// }
//
// void UPointCloudRenderComponent::SetVisiblePoints(const TArray<FVector>& InPoints)
// {
//     // Clear any existing visible points
//     Points.RemoveAll([](const FPointData& Point) {
//         return Point.Color.R < Point.Color.G; // Green points are visible
//     });
//     
//     // Add new visible points
//     for (const FVector& Point : InPoints)
//     {
//         Points.Add(FPointData(Point, FColor::Green, VisiblePointSize));
//     }
//     
//     // Update bounds
//     FBox BoundingBox(EForceInit::ForceInit);
//     for (const FPointData& Point : Points)
//     {
//         BoundingBox += Point.Position;
//     }
//     
//     // Add padding for point size
//     BoundingBox = BoundingBox.ExpandBy(FMath::Max(VisiblePointSize, InvisiblePointSize) * 2.0f);
//     Bounds = FBoxSphereBounds(BoundingBox);
//     
//     MarkRenderStateDirty();
// }
//
// void UPointCloudRenderComponent::SetInvisiblePoints(const TArray<FVector>& InPoints)
// {
//     // Clear any existing invisible points
//     Points.RemoveAll([](const FPointData& Point) {
//         return Point.Color.R > Point.Color.G; // Red points are invisible
//     });
//     
//     // Add new invisible points
//     for (const FVector& Point : InPoints)
//     {
//         Points.Add(FPointData(Point, FColor::Red, InvisiblePointSize));
//     }
//     
//     // Update bounds
//     FBox BoundingBox(EForceInit::ForceInit);
//     for (const FPointData& Point : Points)
//     {
//         BoundingBox += Point.Position;
//     }
//     
//     // Add padding for point size
//     BoundingBox = BoundingBox.ExpandBy(FMath::Max(VisiblePointSize, InvisiblePointSize) * 2.0f);
//     Bounds = FBoxSphereBounds(BoundingBox);
//     
//     MarkRenderStateDirty();
// }
//
// void UPointCloudRenderComponent::ClearPoints()
// {
//     Points.Empty();
//     Bounds = FBoxSphereBounds(FVector::ZeroVector, FVector::ZeroVector, 0.0f);
//     MarkRenderStateDirty();
// }
//
// FPrimitiveSceneProxy* UPointCloudRenderComponent::CreateSceneProxy()
// {
//     if (Points.Num() > 0)
//     {
//         return new FPointCloudSceneProxy(this);
//     }
//     return nullptr;
// }
//
// FBoxSphereBounds UPointCloudRenderComponent::CalcBounds(const FTransform& LocalToWorld) const
// {
//     return Bounds.TransformBy(LocalToWorld);
// }