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

#include "Simulation/PathPlanner.h"

void UPathPlanner::SemiSphericalPath(
    const TArray<FMeshInfo>& InMeshes, const float Radius,
    const int32 NumPoses, const float& SafeHeight,
    TArray<FVector>& OutPositions, TArray<FRotator>& OutRotations)
{
    // Clear output arrays
    OutPositions.Empty(NumPoses);
    OutRotations.Empty(NumPoses);
    
    // If no meshes provided, return early
    if (InMeshes.Num() == 0)
    {
        return;
    }
    
    // Calculate combined bounds of all meshes
    FBoxSphereBounds CombinedBounds = InMeshes[0].Bounds;
    for (int32 i = 1; i < InMeshes.Num(); ++i)
    {
        CombinedBounds = CombinedBounds + InMeshes[i].Bounds;
    }
    
    // Get the center of the combined bounds
    FVector MeshCenter = CombinedBounds.Origin;
    
    // Create a look-at point with z=0 (at the base of the object)
    FVector LookAtPoint = FVector(MeshCenter.X, MeshCenter.Y, SafeHeight);
    
    // Use the golden spiral method to distribute points on a hemisphere
    const double GoldenRatio = (1.0 + FMath::Sqrt(5.0)) / 2.0;
    
    for (int32 i = 0; i < NumPoses; ++i)
    {
        // Normalized index between 0 and 1
        double t = static_cast<double>(i) / static_cast<double>(FMath::Max(1, NumPoses - 1));
        
        // Inclination (phi): 0 at north pole, PI/2 at equator
        // We only want the top hemisphere, so phi ranges from 0 to PI/2
        double phi = FMath::Acos(1.0 - t);
        
        // Azimuth (theta): spiral around based on golden ratio
        double theta = 2.0 * PI * i / GoldenRatio;
        
        // Convert to Cartesian coordinates
        double x = Radius * FMath::Sin(phi) * FMath::Cos(theta);
        double y = Radius * FMath::Sin(phi) * FMath::Sin(theta);
        double z = Radius * FMath::Cos(phi);
        
        // Create position vector (still centered around the mesh center)
        FVector Position = LookAtPoint + FVector(x, y, z);
        
        // Calculate rotation to look at the point with z=0
        FVector LookDirection = (LookAtPoint - Position).GetSafeNormal();
        FRotator Rotation = LookDirection.Rotation();
        
        // Add to output arrays
        OutPositions.Add(Position);
        OutRotations.Add(Rotation);
    }
}

void UPathPlanner::VisDebugPath(
    const TArray<FVector>& InPositions,
    const TArray<FRotator>& InRotations,
    const float Duration,
    const FColor& Color)
{
    if (InPositions.Num() == 0 || InRotations.Num() == 0)
    {
        return;
    }
    
    // Get the world
    UWorld* World = nullptr;
    
    // Try to get the world from the game instance
    if (GEngine && GEngine->GameViewport)
    {
        World = GEngine->GameViewport->GetWorld();
    }
    
    // If we still don't have a world, try to get it from the first available world context
    if (!World && GEngine && GEngine->GetWorldContexts().Num() > 0)
    {
        World = GEngine->GetWorldContexts()[0].World();
    }
    
    if (!World)
    {
        return;
    }
    
    // Draw each position and rotation
    for (int32 i = 0; i < FMath::Min(InPositions.Num(), InRotations.Num()); ++i)
    {
        const FVector Position = InPositions[i];
        const FRotator Rotation = InRotations[i];
        
        // Draw a sphere at the position
        DrawDebugSphere(World, Position, 10.0f, 8, Color, false, Duration, 0, 2.0f);
        
        // Draw coordinate axes to represent rotation
        const float AxisLength = 50.0f;
        
        // Get the basis vectors from the rotator
        const FVector ForwardVector = Rotation.Vector();
        const FVector RightVector = FRotationMatrix(Rotation).GetScaledAxis(EAxis::Y);
        const FVector UpVector = FRotationMatrix(Rotation).GetScaledAxis(EAxis::Z);
        
        // Draw the three axes of the rotation
        DrawDebugLine(World, Position, Position + ForwardVector * AxisLength, FColor::Red, false, Duration, 0, 2.0f);
        DrawDebugLine(World, Position, Position + RightVector * AxisLength, FColor::Green, false, Duration, 0, 2.0f);
        DrawDebugLine(World, Position, Position + UpVector * AxisLength, FColor::Blue, false, Duration, 0, 2.0f);
        
        // Connect positions with lines to show the path
        if (i > 0)
        {
            DrawDebugLine(World, InPositions[i - 1], Position, Color, false, Duration, 0, 1.0f);
        }
    }
}