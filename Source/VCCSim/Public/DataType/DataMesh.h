#pragma once

struct FCompactVertex {
	float X, Y, Z;
};

struct FMeshHeader {
	uint32 Magic;
	uint32 Version;
	uint32 VertexCount;
	uint32 IndexCount;
	uint32 Flags;
};