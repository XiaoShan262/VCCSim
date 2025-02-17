#include "Utils/LoadObj.h"
#include "ProceduralMeshComponent.h"

AObjLoader::AObjLoader()
{
	PrimaryActorTick.bCanEverTick = false;
	MeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("MeshComponent"));
	SetRootComponent(MeshComponent);
}

void AObjLoader::BeginPlay()
{
	Super::BeginPlay();
		
	if (!MeshComponent)
	{
		UE_LOG(LogTemp, Error,
			TEXT("MeshHandler Failed to create ProceduralMeshComponent"));
		return;
	}
	MeshComponent->SetUsingAbsoluteLocation(true);
	MeshComponent->SetUsingAbsoluteRotation(true);
	MeshComponent->SetUsingAbsoluteScale(true);
	MeshComponent->SetWorldTransform(FTransform::Identity);
    
	MeshComponent->RegisterComponent();
    
	MeshComponent->SetVisibility(true);
	MeshComponent->bVisibleInSceneCaptureOnly = false;
    
	// Optimize for frequent updates
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	MeshComponent->SetTranslucentSortPriority(100);
	MeshComponent->SetCastShadow(false);
	MeshComponent->SetMobility(EComponentMobility::Movable);
	MeshComponent->bUseAsyncCooking = true;
	MeshComponent->bReceivesDecals = false;

	if (Material)
	{
		MeshComponent->SetMaterial(0, Material);
	}
	else
	{
		UE_LOG(LogTemp, Error,
			TEXT("MeshHandler Failed to load material"));
	}

	LoadObjFiles();
}

void AObjLoader::LoadObjFiles()
{
    FString tPath = UTF8_TO_TCHAR("C:/GitProjects/RatSimClient/hull");
    
    for (int i = 0; i <= 4; i++)
    {
        FString tFile = tPath + FString::Printf(TEXT("/%d.obj"), i);
        FString tContent;
        
        if (!FFileHelper::LoadFileToString(tContent, *tFile))
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to load OBJ file: %s"), *tFile);
            continue;
        }

        TArray<FString> tLines;
        tContent.ParseIntoArray(tLines, TEXT("\n"), true);
        
        FMesh tMesh;
        TArray<FVector> tempNormals;
        
        // Maps file vertex indices to our final mesh indices
        TMap<int32, int32> vertexIndexMap;
        
        for (FString& tLine : tLines)
        {
            tLine = tLine.TrimStartAndEnd();
            
            // Skip empty lines and comments
            if (tLine.IsEmpty() || tLine.StartsWith("#"))
                continue;

            TArray<FString> tokens;
            tLine.ParseIntoArray(tokens, TEXT(" "), true);
            
            if (tokens.Num() == 0)
                continue;

            if (tokens[0] == "v" && tokens.Num() >= 4)
            {
                FVector vertex;
                vertex.X = FCString::Atof(*tokens[1]);
                vertex.Y = FCString::Atof(*tokens[2]);
                vertex.Z = FCString::Atof(*tokens[3]);
                tMesh.Vertices.Add(vertex);
            }
            else if (tokens[0] == "vn" && tokens.Num() >= 4)
            {
                FVector normal;
                normal.X = FCString::Atof(*tokens[1]);
                normal.Y = FCString::Atof(*tokens[2]);
                normal.Z = FCString::Atof(*tokens[3]);
                tempNormals.Add(normal);
            }
            else if (tokens[0] == "f" && tokens.Num() >= 4)
            {
                // Process face indices
                for (int32 j = 1; j < 4; j++)
                {
                    TArray<FString> indices;
                    tokens[j].ParseIntoArray(indices, TEXT("/"), true);
                    
                    if (indices.Num() > 0)
                    {
                        int32 vertIdx = FCString::Atoi(*indices[0]) - 1; // OBJ indices are 1-based
                        
                        // Check if vertex index is valid
                        if (vertIdx >= 0 && vertIdx < tMesh.Vertices.Num())
                        {
                            // Add to triangle indices
                            tMesh.Triangles.Add(vertIdx);
                            
                            // If we have normal indices and they're valid
                            if (indices.Num() >= 3 && !indices[2].IsEmpty())
                            {
                                int32 normalIdx = FCString::Atoi(*indices[2]) - 1;
                                if (normalIdx >= 0 && normalIdx < tempNormals.Num())
                                {
                                    // Ensure we have enough space in the normals array
                                    while (tMesh.Normals.Num() <= vertIdx)
                                    {
                                        tMesh.Normals.Add(FVector::ZeroVector);
                                    }
                                    tMesh.Normals[vertIdx] = tempNormals[normalIdx];
                                }
                            }
                        }
                        else
                        {
                            UE_LOG(LogTemp, Warning, TEXT("Invalid vertex index in OBJ file: %d"), vertIdx);
                        }
                    }
                }
            }
        }

        // Ensure we have normals for all vertices
        if (tMesh.Normals.Num() < tMesh.Vertices.Num())
        {
            tMesh.Normals.SetNum(tMesh.Vertices.Num());
            for (int32 vertIndex = 0; vertIndex < tMesh.Vertices.Num(); vertIndex++)
            {
                if (tMesh.Normals[vertIndex].IsZero())
                {
                    tMesh.Normals[vertIndex] = FVector(0, 0, 1);
                }
            }
        }

        if (tMesh.Vertices.Num() > 0 && tMesh.Triangles.Num() > 0)
        {
            Meshes.Add(tMesh);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("Loaded OBJ file contains no valid mesh data: %s"), *tFile);
        }
    }
}

void AObjLoader::UpdateMesh()
{
	static int index = 0;
	if (index >= Meshes.Num())
	{
		index = 0;
	}

	MeshComponent->ClearAllMeshSections();
	
	FMesh tMesh = Meshes[index];
	MeshComponent->CreateMeshSection_LinearColor(0,
		tMesh.Vertices,
		tMesh.Triangles,
		tMesh.Normals,
		TArray<FVector2D>(),
		TArray<FLinearColor>(),
		TArray<FProcMeshTangent>(),
		false);
	index++;
}
