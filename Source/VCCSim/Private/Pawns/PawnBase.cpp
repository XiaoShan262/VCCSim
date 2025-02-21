#include "Pawns/PawnBase.h"

void APawnBase::SetRecorder(ARecorder* InRecorder)
{
	Recorder = InRecorder;
}

void APawnBase::SetRecordInterval(const float& Interval)
{
	RecordInterval = Interval;
	bRecorded = true;
}