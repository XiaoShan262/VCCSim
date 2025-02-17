#include "Sensors/SensorFactory.h"
#include "Sensors/LidarSensor.h"
#include "Sensors/DepthCamera.h"
#include "Sensors/CameraSensor.h"

UPrimitiveComponent* FSensorFactory::CreateSensor(
	ESensorType SensorType, AActor* Owner, FName Name, const SensorConfig& Config)
{
	if (!Owner)
	{
		UE_LOG(LogTemp, Error, TEXT("No owner found!"));
		return nullptr;
	}

	UPrimitiveComponent* Sensor = nullptr;

	switch (SensorType)
	{
	case ESensorType::Lidar:
		{
			const LiDARConfig* LidarConfig =
				static_cast<const LiDARConfig*>(&Config);
			if (!LidarConfig)
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Invalid config type for Lidar sensor!"));
				return nullptr;
			}
            
			if (auto LidarSensor = NewObject<ULidarComponent>(Owner, Name))
			{
				Sensor = LidarSensor;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Failed to create Lidar sensor!"));
			}
			break;
		}
	case ESensorType::DepthCamera:
		{
			const DepthCameraConfig* DepthConfig =
				static_cast<const DepthCameraConfig*>(&Config);
			if (!DepthConfig)
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Invalid config type for Depth Camera sensor!"));
				return nullptr;
			}

			if (auto DepthSensor = NewObject<UDepthCameraComponent>(Owner, Name))
			{
				Sensor = DepthSensor;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Failed to create Depth Camera sensor!"));
			}
			break;
		}
	case ESensorType::RGBCamera:
		{
			const RGBCameraConfig* CameraConfig =
				static_cast<const RGBCameraConfig*>(&Config);
			if (!CameraConfig)
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Invalid config type for RGB Camera sensor!"));
				return nullptr;
			}

			if (auto CameraSensor = NewObject<URGBCameraComponent>(Owner, Name))
			{
				Sensor = CameraSensor;
			}
			else
			{
				UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
								"Failed to create RGB Camera sensor!"));
			}
			
			break;
		}
	default:
		{
			UE_LOG(LogTemp, Error, TEXT("FSensorFactory::CreateSensor: "
							"Invalid sensor type!"));
		}
	}

	if (Sensor)
	{
		Sensor->RegisterComponent();
	}

	return Sensor;
}