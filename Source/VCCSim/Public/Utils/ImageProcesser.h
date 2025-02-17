#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"

class FAsyncImageSaveTask : public FNonAbandonableTask
{
public:
	FAsyncImageSaveTask(const TArray<FColor>& InPixels, FIntPoint InSize, 
		const FString& InFilePath)
		: Pixels(InPixels)
		, Size(InSize)
		, FilePath(InFilePath)
	{}

	void DoWork();

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FAsyncImageSaveTask,
			STATGROUP_ThreadPoolAsyncTasks);
	}

private:
	TArray<FColor> Pixels;
	FIntPoint Size;
	FString FilePath;
};