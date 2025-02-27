#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "ProceduralMeshComponent.h"
#include "MeshManager.generated.h"

struct FVCCSimConfig;
struct FMeshHeader;
struct FCompactVertex;

// Structure to hold mesh data for processing
USTRUCT()
struct FMeshBufferData
{
    GENERATED_BODY()

    TArray<FVector> Vertices;
    TArray<FVector> Normals;
    TArray<int32> Triangles;
    TArray<FVector2D> UVs;
    FTransform Transform;
    bool bNeedsUpdate;
    FThreadSafeBool* ProcessingTaskComplete;

    FMeshBufferData() : bNeedsUpdate(false), ProcessingTaskComplete(nullptr) {}
    ~FMeshBufferData()
    {
        if (ProcessingTaskComplete)
        {
            delete ProcessingTaskComplete;
            ProcessingTaskComplete = nullptr;
        }
    }
};

// Structure to track each mesh instance
USTRUCT()
struct FMeshInstance
{
    GENERATED_BODY()

    UPROPERTY()
    UProceduralMeshComponent* MeshComponent;
    
    FMeshBufferData MeshBuffers[2];
    int32 CurrentBufferIndex;
    bool bIsFirstMesh;
    bool bIsActive;

    FMeshInstance() 
        : MeshComponent(nullptr)
        , CurrentBufferIndex(0)
        , bIsFirstMesh(true)
        , bIsActive(false) 
    {}
};

UCLASS()
class VCCSIM_API UFMeshManager : public UObject
{
    GENERATED_BODY()

public:
    UFMeshManager();
    void RConfigure(const FVCCSimConfig& Config);

    // GRPC Call interfaces
    int32 AddGlobalMesh();
    bool RemoveGlobalMesh(int32 MeshID);
    bool UpdateMesh(int32 MeshID, const uint8* MeshData,
        uint32 DataSize, const FTransform& Transform);
    
    // Clear all meshes (useful for cleanup)
    void ClearAllMeshes();

    // Material used for all meshes
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VCCSim|MeshManager")
    UMaterialInterface* MeshMaterial;
    
    // Update interval for mesh rendering (in seconds)
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "VCCSim|MeshManager")
    float MeshUpdateInterval = 1.f / 30.f;

    bool ProcessMeshData(const uint8* MeshData, uint32 DataSize, 
    TArray<FVector>& OutVertices, TArray<FVector>& OutNormals, 
    TArray<int32>& OutTriangles);

protected:
    UPROPERTY()
    TMap<int32, UProceduralMeshComponent*> MeshComponents;
    TMap<int32, TSharedPtr<FMeshInstance>> MeshInstanceData;
    TMap<int32, FCriticalSection*> MeshUpdateLocks;
    
    // Counter for generating unique mesh IDs
    int32 NextMeshID = 1;
    
    // Thread safety for mesh operations
    FCriticalSection MeshMapLock;
    
    UPROPERTY()
    AActor* OwnerActor = nullptr;
    FTimerHandle UpdateTimerHandle;
    
    UProceduralMeshComponent* CreateProceduralMeshComponent();
    
    FMeshInstance* FindMeshInstance(int32 MeshID);
    
    // Internal update method called by timer
    void UpdateMeshesInternal();
    
    void CalculateVertexNormals(const TArray<FVector>& InVertices, 
        const TArray<int32>& InTriangles, TArray<FVector>& OutNormals);

private:
    int32 AddGlobalMeshInternal();
    bool RemoveGlobalMeshInternal(int32 MeshID);
};

class FMeshProcessingWorker : public FNonAbandonableTask
{
public:
    FMeshProcessingWorker(const uint8* InMeshData, uint32 InDataSize, 
                        TArray<FVector>* OutVertices, TArray<FVector>* OutNormals, 
                        TArray<int32>* OutTriangles, UFMeshManager* InManager,
                        int32 InMeshID, FThreadSafeBool* InProcessingComplete)
        : MeshData(InMeshData)
        , DataSize(InDataSize)
        , Vertices(OutVertices)
        , Normals(OutNormals)
        , Triangles(OutTriangles)
        , Manager(InManager)
        , MeshID(InMeshID)
        , ProcessingComplete(InProcessingComplete)
    {
        // Make a deep copy of the mesh data to avoid lifetime issues
        DataCopy.AddUninitialized(DataSize);
        FMemory::Memcpy(DataCopy.GetData(), MeshData, DataSize);
        MeshData = DataCopy.GetData();
    }

    void DoWork()
    {
        // Process the mesh data on the task thread
        Manager->ProcessMeshData(MeshData, DataSize, *Vertices,
            *Normals, *Triangles);
        *ProcessingComplete = true;
    }

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(FMeshProcessingWorker,
            STATGROUP_ThreadPoolAsyncTasks);
    }

private:
    const uint8* MeshData;
    uint32 DataSize;
    TArray<uint8> DataCopy;
    TArray<FVector>* Vertices;
    TArray<FVector>* Normals;
    TArray<int32>* Triangles;
    UFMeshManager* Manager;
    int32 MeshID;
    FThreadSafeBool* ProcessingComplete;
};