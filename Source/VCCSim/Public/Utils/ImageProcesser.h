﻿/*
* Copyright (C) 2025 Visual Computing Research Center, Shenzhen University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include "CoreMinimal.h"
#include "Async/AsyncWork.h"

class FAsyncImageSaveTask : public FNonAbandonableTask
{
public:
	FAsyncImageSaveTask(
		const TArray<FColor>& InPixels,
		FIntPoint InSize, const FString& InFilePath)
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