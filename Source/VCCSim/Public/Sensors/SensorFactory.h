#pragma once

#include "SensorBase.h"

class FSensorFactory
{
public:

	// This function creates a sensor component based on the sensor type.
	// Attention: This function is not responsible for configure the sensor.
	static UPrimitiveComponent* CreateSensor(
		ESensorType SensorType, AActor* Owner, FName Name,
		const SensorConfig& Config);
};
