#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PawnBase.generated.h"

class ARecorder;

enum class EPawnType : uint8
{
	Drone,
	Car,
	Flash
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
	UFUNCTION()
	void SetRecordState(bool RState){ RecordState = RState; }

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VCCSim|Debug")
	bool bRecorded = false;
	bool RecordState = false;

	UPROPERTY()
	ARecorder* Recorder;
	float RecordInterval = -1.f;
	float TimeSinceLastCapture = 0.f;
};