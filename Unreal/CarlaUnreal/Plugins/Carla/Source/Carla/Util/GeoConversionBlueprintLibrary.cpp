// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT/>.

#include "Carla/Util/GeoConversionBlueprintLibrary.h"

#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/MapGen/LargeMapManager.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/GeoProjection.h"
#include "carla/geom/GeoLocation.h"
#include "carla/geom/Location.h"
#include <util/enable-ue4-macros.h>

bool UGeoConversionBlueprintLibrary::UEToGeoLocation(
    const UObject *WorldContextObject,
    const FVector &WorldLocation,
    FGeoLocation &OutGeoLocation)
{
  if (!WorldContextObject)
  {
    return false;
  }

  // 1. Get the episode and its geo projection.
  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(WorldContextObject);
  if (!Episode)
  {
    UE_LOG(LogTemp, Warning,
        TEXT("UEToGeoLocation: CarlaEpisode not available. Is the CARLA game running?"));
    return false;
  }
  const carla::geom::GeoProjection &GeoProj = Episode->GetGeoProjection();

  // 2. Convert local → global if LargeMapManager is active.
  FVector GlobalLocation = WorldLocation;
  ALargeMapManager *LargeMap = UCarlaStatics::GetLargeMapManager(WorldContextObject);
  if (LargeMap)
  {
    GlobalLocation = LargeMap->LocalToGlobalLocation(WorldLocation);
  }

  // 3. FVector (cm) → carla::geom::Location (m) → GeoLocation.
  carla::geom::Location CarlaLocation(GlobalLocation);
  carla::geom::GeoLocation GeoLoc = GeoProj.TransformToGeoLocation(CarlaLocation);

  OutGeoLocation.Latitude  = GeoLoc.latitude;
  OutGeoLocation.Longitude = GeoLoc.longitude;
  OutGeoLocation.Altitude  = GeoLoc.altitude;
  return true;
}

bool UGeoConversionBlueprintLibrary::GeoToUELocation(
    const UObject *WorldContextObject,
    const FGeoLocation &GeoLocation,
    FVector &OutWorldLocation)
{
  if (!WorldContextObject)
  {
    return false;
  }

  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(WorldContextObject);
  if (!Episode)
  {
    UE_LOG(LogTemp, Warning,
        TEXT("GeoToUELocation: CarlaEpisode not available. Is the CARLA game running?"));
    return false;
  }
  const carla::geom::GeoProjection &GeoProj = Episode->GetGeoProjection();

  // GeoLocation (lat/lon/alt) → carla::geom::Location (m).
  carla::geom::GeoLocation CarlaGeoLoc(GeoLocation.Latitude, GeoLocation.Longitude, GeoLocation.Altitude);
  carla::geom::Location CarlaLocation = GeoProj.GeoLocationToTransform(CarlaGeoLoc);

  // carla::geom::Location (m) → FVector (cm) via the conversion operator.
  FVector GlobalLocation = CarlaLocation;

  // Global → local if LargeMapManager is active.
  ALargeMapManager *LargeMap = UCarlaStatics::GetLargeMapManager(WorldContextObject);
  if (LargeMap)
  {
    GlobalLocation = LargeMap->GlobalToLocalLocation(GlobalLocation);
  }

  OutWorldLocation = GlobalLocation;
  return true;
}

bool UGeoConversionBlueprintLibrary::GetActorGeoLocation(
    const UObject *WorldContextObject,
    const AActor *Actor,
    FGeoLocation &OutGeoLocation)
{
  if (!Actor)
  {
    return false;
  }
  return UEToGeoLocation(WorldContextObject, Actor->GetActorLocation(), OutGeoLocation);
}
