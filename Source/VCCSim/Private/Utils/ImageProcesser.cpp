/*
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

#include "Utils/ImageProcesser.h"
#include "ImageUtils.h"

void FAsyncImageSaveTask::DoWork()
{
	TArray64<uint8> CompressedBitmap;
	FImageUtils::PNGCompressImageArray(Size.X, Size.Y, Pixels,
		CompressedBitmap);

	if (FFileHelper::SaveArrayToFile(CompressedBitmap, *FilePath))
	{
		UE_LOG(LogTemp, Warning, TEXT("Render target saved to: %s"), *FilePath);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Failed to save render target to file."));
	}
}
