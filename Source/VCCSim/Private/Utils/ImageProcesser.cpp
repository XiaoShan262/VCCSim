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
