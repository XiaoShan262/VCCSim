#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PawnBase.generated.h"

class ARecorder;

enum class EPawnType : uint8
{
	Drone,
	Car
};

UCLASS()
class VCCSIM_API APawnBase : public APawn
{
	GENERATED_BODY()
	
public:
	UFUNCTION()
	void SetRecorder(ARecorder* InRecorder);
	UFUNCTION()
	void SetRecordInterval(const float& Interval);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "RatSim|Debug")
	bool bRecorded = false;

	UPROPERTY()
	ARecorder* Recorder;
	float RecordInterval = -1.f;
	float TimeSinceLastCapture = 0.f;
};