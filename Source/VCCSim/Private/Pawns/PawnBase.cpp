#include "Pawns/PawnBase.h"
#include "Simulation/Recorder.h"

void APawnBase::SetRecorder(ARecorder* InRecorder)
{
	Recorder = InRecorder;
	RecordState = InRecorder->RecordState;
	InRecorder->OnRecordStateChanged.AddDynamic(this, &APawnBase::SetRecordState);
}

void APawnBase::SetRecordInterval(const float& Interval)
{
	RecordInterval = Interval;
	bRecorded = true;
}