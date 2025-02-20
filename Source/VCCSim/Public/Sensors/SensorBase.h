#pragma once

enum class ESensorType : uint8
{
	Lidar,
	DepthCamera,
	RGBCamera
};

class FSensorConfig
{
public:
	float RecordInterval = 0.2f;
};

struct FVCCSimOdom
{
	FVector Location = FVector::ZeroVector;
	FRotator Rotation = FRotator::ZeroRotator;
	FVector LinearVelocity = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
};