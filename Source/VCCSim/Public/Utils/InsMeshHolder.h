#pragma once
#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Materials/MaterialInstanceDynamic.h"

#include "InsMeshHolder.generated.h"


UCLASS()
class VCCSIM_API UInsMeshHolder : public USceneComponent
{
	GENERATED_BODY()

public:
	UInsMeshHolder();
	virtual void OnRegister() override;
	void CreateStaticMeshes();
	void ClearAndAddNewInstances(const TArray<FTransform>& Transforms) const;
	void ClearAndAddNewInstancesWithColors(const TArray<FTransform>& Transforms, 
		const TArray<FColor>& Colors) const;

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;
    
	// Thread-safe method to queue updates
	void QueueInstanceUpdate(const TArray<FTransform>& Transforms,
		const TArray<FColor>& Colors);
	
	// Thread-safe queue for pending updates
	FCriticalSection UpdateLock;
	TQueue<TPair<TArray<FTransform>, TArray<FColor>>> PendingUpdates;

	UInstancedStaticMeshComponent* GetInstancedMeshComponent() const;
	UInstancedStaticMeshComponent* GetInstancedMeshComponentColor() const;

private:
	UPROPERTY()
	UInstancedStaticMeshComponent* InstancedMeshComponent = nullptr;
	UPROPERTY()
	UInstancedStaticMeshComponent* InstancedMeshComponentColor = nullptr;
	float IMCCTimer = 0.f;
	UPROPERTY()
	UMaterialInstanceDynamic* ColorMaterialInstance = nullptr;
	
};