// // SafeZoneManager.cpp
// #include "Simulation/SafeZoneManager.h"
// #include "Engine/World.h"
// #include "DistanceFieldAtlas.h"
// #include "GlobalDistanceField.h"
// #include "Rendering/DistanceFieldVolume.h"
// #include "Async/ParallelFor.h"
// #include "GameFramework/Actor.h"
// #include "Components/PrimitiveComponent.h"
// #include "DrawDebugHelpers.h"
//
// // Additional member variables to improve efficiency
// struct FSafeZoneManager_ThreadSafeData
// {
//     TMap<FIntVector, float> DistanceFieldCache;
//     TArray<TPair<FVector, float>> DynamicObstaclePositions;
//     FDateTime LastFullUpdate;
//     FCriticalSection CacheLock;
// };
//
// // Static data for thread safety across instances
// static TSharedPtr<FSafeZoneManager_ThreadSafeData> ThreadSafeData;
//
// void USafeZoneManager::Initialize()
// {
//     UWorld* World = GetWorld();
//     if (!World)
//         return;
//
//     // Initialize thread-safe data if not already done
//     if (!ThreadSafeData.IsValid())
//     {
//         ThreadSafeData = MakeShared<FSafeZoneManager_ThreadSafeData>();
//         ThreadSafeData->LastFullUpdate = FDateTime::Now();
//     }
//     
//     // Access UE5's global distance field
//     WorldDistanceField = World->Scene->dis
//     
//     // Set up update timer for dynamic obstacles
//     FTimerHandle UpdateTimerHandle;
//     World->GetTimerManager().SetTimer(UpdateTimerHandle, 
//         FTimerDelegate::CreateUObject(this, &USafeZoneManager::UpdateDynamicObstaclesTimerCallback), 
//         UpdateFrequency, true);
//         
//     UE_LOG(LogTemp, Log, TEXT("SafeZoneManager initialized with drone safety distance: %f, car safety distance: %f"), 
//            SafeDistanceDrone, SafeDistanceCar);
// }
//
// bool USafeZoneManager::IsPositionSafeDrone(const FVector& Position)
// {
//     // Thread-safe access
//     FScopeLock Lock(&SafetyCheckLock);
//     
//     // Height check for drones
//     float TerrainHeight = GetTerrainHeightAtLocation(Position);
//     if (Position.Z - TerrainHeight < SafeHeight)
//     {
//         return false; // Too close to ground
//     }
//     
//     // Check against both static and dynamic obstacles
//     float MinDistance = GetCombinedDistanceField(Position);
//     
//     // Position is safe if distance exceeds drone safety distance
//     return MinDistance > SafeDistanceDrone;
// }
//
// bool USafeZoneManager::IsPositionSafeCar(const FVector& Position)
// {
//     // Thread-safe access
//     FScopeLock Lock(&SafetyCheckLock);
//     
//     // Cars only move on ground, so we need to check if position is on valid terrain
//     if (!IsValidTerrainPosition(Position))
//     {
//         return false;
//     }
//     
//     // Check against both static and dynamic obstacles
//     float MinDistance = GetCombinedDistanceField(Position);
//     
//     // Position is safe if distance exceeds car safety distance
//     return MinDistance > SafeDistanceCar;
// }
//
// float USafeZoneManager::GetSafetyMargin(const FVector& Position)
// {
//     // Thread-safe access
//     FScopeLock Lock(&SafetyCheckLock);
//     
//     // Return the raw distance to nearest obstacle
//     return GetCombinedDistanceField(Position);
// }
//
// void USafeZoneManager::UpdateDynamicObstacles(TArray<AActor*> DynamicActors)
// {
//     // Skip update if too frequent
//     static double LastUpdateTime = 0;
//     double CurrentTime = FPlatformTime::Seconds();
//     if (CurrentTime - LastUpdateTime < UpdateFrequency)
//         return;
//     
//     LastUpdateTime = CurrentTime;
//     
//     // Use a thread-safe approach to update dynamic obstacle data
//     FScopeLock DataLock(&ThreadSafeData->CacheLock);
//     
//     // Clear previous dynamic obstacle data
//     ThreadSafeData->DynamicObstaclePositions.Empty();
//     
//     // Process all dynamic actors
//     for (AActor* Actor : DynamicActors)
//     {
//         if (!Actor || !Actor->IsValidLowLevel())
//             continue;
//             
//         // Get actor bounds
//         FBox ActorBounds = Actor->GetComponentsBoundingBox(true);
//         
//         // Store the actor position and approximate radius for fast distance checking
//         float BoundsRadius = ActorBounds.GetExtent().GetMax();
//         ThreadSafeData->DynamicObstaclePositions.Add(TPair<FVector, float>(Actor->GetActorLocation(), BoundsRadius));
//     }
//     
//     // Every 10 seconds, refresh the entire distance field cache
//     if ((FDateTime::Now() - ThreadSafeData->LastFullUpdate).GetTotalSeconds() > 10.0)
//     {
//         ThreadSafeData->DistanceFieldCache.Empty();
//         ThreadSafeData->LastFullUpdate = FDateTime::Now();
//     }
// }
//
// // Timer callback for dynamic updates
// void USafeZoneManager::UpdateDynamicObstaclesTimerCallback()
// {
//     UWorld* World = GetWorld();
//     if (!World)
//         return;
//         
//     // Gather all relevant dynamic actors
//     TArray<AActor*> DynamicActors;
//     
//     // You can filter by tag or class here, for example:
//     // for (TActorIterator<AActor> It(World); It; ++It)
//     // {
//     //     if (It->ActorHasTag(TEXT("DynamicObstacle")))
//     //     {
//     //         DynamicActors.Add(*It);
//     //     }
//     // }
//     
//     // For better performance, maintain a list of subscribed actors instead of gathering all
//     
//     UpdateDynamicObstacles(DynamicActors);
// }
//
// // Helper function to check if position is on valid terrain for cars
// bool USafeZoneManager::IsValidTerrainPosition(const FVector& Position)
// {
//     UWorld* World = GetWorld();
//     if (!World)
//         return false;
//         
//     // Simple raycast to detect terrain
//     FHitResult HitResult;
//     FVector Start = Position + FVector(0, 0, 100);
//     FVector End = Position - FVector(0, 0, 100);
//     
//     FCollisionQueryParams QueryParams;
//     QueryParams.bTraceComplex = false;
//     
//     // Only check against static landscape/terrain
//     if (World->LineTraceSingleByChannel(HitResult, Start, End, ECC_WorldStatic, QueryParams))
//     {
//         // Check if hit component is a terrain type
//         if (HitResult.Component.IsValid())
//         {
//             // You might want more sophisticated terrain detection based on your game
//             return true;
//         }
//     }
//     
//     return false;
// }
//
// // Helper function to get terrain height at location
// float USafeZoneManager::GetTerrainHeightAtLocation(const FVector& Position)
// {
//     UWorld* World = GetWorld();
//     if (!World)
//         return -10000.0f; // Some very low value as fallback
//         
//     FHitResult HitResult;
//     FVector Start = Position + FVector(0, 0, 10000);
//     FVector End = Position - FVector(0, 0, 10000);
//     
//     FCollisionQueryParams QueryParams;
//     QueryParams.bTraceComplex = false;
//     
//     if (World->LineTraceSingleByChannel(HitResult, Start, End, ECC_WorldStatic, QueryParams))
//     {
//         return HitResult.ImpactPoint.Z;
//     }
//     
//     return -10000.0f; // Fallback if no terrain found
// }
//
// // Optimized helper function for combined distance field sampling
// float USafeZoneManager::GetCombinedDistanceField(const FVector& Position)
// {
//     // Check static distance field
//     float StaticDistance = SampleWorldDistanceField(Position);
//     
//     // Early optimization - if we're already too close to static geometry, return
//     if (StaticDistance < FMath::Min(SafeDistanceDrone, SafeDistanceCar) * 0.5f)
//     {
//         return StaticDistance;
//     }
//     
//     // Check dynamic obstacles using our optimized representation
//     float DynamicDistance = MAX_flt;
//     
//     // Thread-safe access to dynamic obstacles
//     FScopeLock DataLock(&ThreadSafeData->CacheLock);
//     
//     for (const TPair<FVector, float>& ObstaclePair : ThreadSafeData->DynamicObstaclePositions)
//     {
//         // Fast approximate distance check
//         float Distance = FVector::Distance(Position, ObstaclePair.Key) - ObstaclePair.Value;
//         DynamicDistance = FMath::Min(DynamicDistance, Distance);
//         
//         // Early out if already unsafe
//         if (DynamicDistance < FMath::Min(SafeDistanceDrone, SafeDistanceCar) * 0.5f)
//         {
//             break;
//         }
//     }
//     
//     // Return minimum distance (closest obstacle)
//     return FMath::Min(StaticDistance, DynamicDistance);
// }
//
// // Cached distance field sampling with spatial hashing
// float USafeZoneManager::SampleWorldDistanceField(const FVector& Position)
// {
//     // Skip if no world distance field
//     if (!WorldDistanceField)
//         return MAX_flt;
//     
//     // Discretize position to reduce number of actual samples
//     // This creates a spatial hash that caches distance field values
//     float CellSize = 50.0f; // Adjust based on precision needs
//     FIntVector CellPos(
//         FMath::FloorToInt(Position.X / CellSize),
//         FMath::FloorToInt(Position.Y / CellSize),
//         FMath::FloorToInt(Position.Z / CellSize)
//     );
//     
//     // Try to find in cache first
//     FScopeLock DataLock(&ThreadSafeData->CacheLock);
//     if (float* CachedDistance = ThreadSafeData->DistanceFieldCache.Find(CellPos))
//     {
//         return *CachedDistance;
//     }
//     
//     // If not in cache, sample the distance field
//     float Distance;
//     
//     // Use UE5's global distance field sampling function
//     FVector LocalPosition = Position;
//     WorldDistanceField->VolumeToWorld.InverseTransformPosition(LocalPosition);
//     
//     // Sample the distance field
//     FVector3f LocalPositionFloat(LocalPosition);
//     float DistanceFieldValue = WorldDistanceField->Data.SampleLinear(
//         LocalPositionFloat.X, 
//         LocalPositionFloat.Y, 
//         LocalPositionFloat.Z);
//     
//     // Convert the normalized value to world units
//     Distance = DistanceFieldValue * WorldDistanceField->VolumeToWorld.GetScale3D().GetMax();
//     
//     // Cache the result
//     ThreadSafeData->DistanceFieldCache.Add(CellPos, Distance);
//     
//     return Distance;
// }
//
// // Visualize safe zone (for debugging)
// void USafeZoneManager::VisualizeSafeZone(UWorld* World, const FVector& Center, float Radius, bool IsDrone)
// {
//     if (!World)
//         return;
//         
//     const int32 NumSamples = 200;
//     const float SafeDistance = IsDrone ? SafeDistanceDrone : SafeDistanceCar;
//     
//     for (int32 i = 0; i < NumSamples; ++i)
//     {
//         FVector RandomDirection = FMath::VRand();
//         FVector SamplePoint = Center + RandomDirection * FMath::RandRange(0.0f, Radius);
//         
//         bool IsSafe = IsDrone ? 
//             IsPositionSafeDrone(SamplePoint) : 
//             IsPositionSafeCar(SamplePoint);
//             
//         FColor DebugColor = IsSafe ? FColor::Green : FColor::Red;
//         DrawDebugPoint(World, SamplePoint, 10.0f, DebugColor, false, 1.0f);
//     }
// }