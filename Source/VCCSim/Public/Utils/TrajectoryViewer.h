#pragma once

#include "CoreMinimal.h"
#include "Components/SplineComponent.h"
#include "Components/SplineMeshComponent.h"
#include "Containers/Deque.h"
#include "GameFramework/Actor.h"
#include "TrajectoryViewer.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class UTrajectoryViewer : public USceneComponent
{
    GENERATED_BODY()
    
public:
    UTrajectoryViewer();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void TickComponent(float DeltaTime, enum ELevelTick TickType,
       FActorComponentTickFunction* ThisTickFunction) override;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    USplineComponent* SplineComponent;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float TraveledDistance = -0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float DisplayDistance = -0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Trajectory")
    float LastTraveledDistance = -250.0f;

    // Path visualization properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Visualization")
    UStaticMesh* PathMesh;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Visualization")
    UMaterialInterface* PathMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Visualization")
    float PathWidth = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Path Visualization")
    float StepSize = 4.0f;
    
private:
    float TotalLength = -0.1f;
    
    TDeque<USplineMeshComponent*> PathMeshes;

    // Helper functions
    void UpdatePartialPath();
    void ClearSplineMeshes();
    FVector GetPointAtDistance(float Distance) const;
    
    // Material instance for dynamic color changes
    UPROPERTY()
    UMaterialInstanceDynamic* ObstacleMaterialInstance;
};