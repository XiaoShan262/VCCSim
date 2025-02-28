# Configuration Guide

## Overview

TRSConfigMOL is a simulation configuration system for Unreal Engine 5, designed for robotic simulations. This document explains the configuration parameters and their usage.

> **Attention!**
>
> - Length unit is in centimeter.
> - Angle unit is in degree.
> - Uses left-handed coordinate system (same as UE5).

## General Configuration (`VCCSimPresets`)

| Parameter | Description | Default Value |
|-----------|-------------|---------------|
| IP | Server IP address for simulation connection | "0.0.0.0" |
| Port | Port number for simulation connection | 50996 |
| MainCharacter | Name or tag of the main character (tag recommended) | "Mavic" |
| StaticMeshActor | Array of names or tags for static mesh actors | ["MFMesh", "CompanionCube"] |
| SubWindows | Array of window types to display | ["Lit", "Unit", "PointCloud"] |
| SubWindowsOpacities | Opacity values for each sub-window | [0.95, 0.85, 0.75] |
| StartWithRecording | Whether to begin recording on startup | false |
| UseMeshManager | Enable mesh management system(MMS) | true |
| MeshMaterial | Path to the material used for MMS meshes | "/VCCSim/Materials/M_Static_mesh" |
| LS_StartOffset | Starting frame offset | 0 |
| BufferSize | Size of the buffer in frames | 100 |
| LogSavePath | Directory path where logs are saved | "C:/UEProjects/VCCSimDev/Plugins/VCCSim/Logs" |
| DefaultDronePawn | Path to the default drone pawn blueprint | "/VCCSim/Pawn/Drone/BP_PreciseDrone_1C_1D_L.BP_PreciseDrone_1C_1D_L_C" |
| DefaultCarPawn | Path to the default car pawn blueprint | "/VCCSim/Pawn/Husky/Blueprints/BP_CarPawn_Husky.BP_CarPawn_Husky_C" |
| DefaultFlashPawn | Path to the default flash pawn blueprint | "/VCCSim/Pawn/Flash/BP_FlashPawn.BP_FlashPawn_C" |

## Robot Configuration

Robots are configured in array format, with each robot having its own specific components.

### Robot General Settings

| Parameter | Description |
|-----------|-------------|
| UETag | The unique tag/identifier for the robot in Unreal Engine |
| Type | Robot type ("Drone", "Car", "Flash") |
| RecordInterval | Time interval between recordings in seconds. Use -1 to disable recording |

### Component Configurations

Each robot can have different sensor components configured:

#### Lidar Component

```toml
[Robots.ComponentConfigs.Lidar]
RecordInterval = 0.5
NumRays = 32
NumPoints = 16000
ScannerRangeInner = 300
ScannerRangeOuter = 3000
ScannerAngleUp = 25
ScannerAngleDown = 25
bVisualizePoints = true
```

| Parameter | Description |
|-----------|-------------|
| RecordInterval | Time interval between lidar captures (seconds) |
| NumRays | Number of rays in the lidar scan |
| NumPoints | Total number of points to capture |
| ScannerRangeInner | Minimum range of the scanner (cm) |
| ScannerRangeOuter | Maximum range of the scanner (cm) |
| ScannerAngleUp | Upper vertical angle of the scanner (degrees) |
| ScannerAngleDown | Lower vertical angle of the scanner (degrees) |
| bVisualizePoints | Whether to visualize lidar points in the simulation |

#### RGB Camera Component

```toml
[Robots.ComponentConfigs.RGBCamera]
RecordInterval = 0.5
FOV = 90
Width = 1500
Height = 800
bOrthographic = false
```

| Parameter | Description |
|-----------|-------------|
| RecordInterval | Time interval between camera captures (seconds) |
| FOV | Field of view in degrees |
| Width | Image width in pixels |
| Height | Image height in pixels |
| bOrthographic | Use orthographic projection instead of perspective |

#### Depth Camera Component

```toml
[Robots.ComponentConfigs.DepthCamera]
FOV = 90
Width = 512
Height = 512
MaxRange = 2000.0
MinRange = 0.0
bOrthographic = false
```

| Parameter | Description |
|-----------|-------------|
| FOV | Field of view in degrees |
| Width | Image width in pixels |
| Height | Image height in pixels |
| MaxRange | Maximum depth range (cm) |
| MinRange | Minimum depth range (cm) |
| bOrthographic | Use orthographic projection instead of perspective |

## Example Configurations

### Drone Example (Mavic)

```toml
[[Robots]]
UETag = "Mavic"
Type = "Drone"
RecordInterval = 0.5

[Robots.ComponentConfigs.RGBCamera]
RecordInterval = 0.5
FOV = 90
Width = 1500
Height = 800
bOrthographic = false
```

### Car Example (Husky)

```toml
[[Robots]]
UETag = "Husky"
Type = "Car"

[Robots.ComponentConfigs.Lidar]
NumRays = 32
NumPoints = 16000
ScannerRangeInner = 100
ScannerRangeOuter = 4000
ScannerAngleUp = 35
ScannerAngleDown = 25
bVisualizePoints = true
```

### Flash Example

```toml
[[Robots]]
UETag = "Flash"
Type = "Flash"
RecordInterval = -1

[Robots.ComponentConfigs.RGBCamera]
RecordInterval = -1
FOV = 90
Width = 800
Height = 600
bOrthographic = false
```

## Notes

- Components with a RecordInterval of -1 will not record data
- Commented sections (lines starting with #) are inactive features that can be uncommented when needed