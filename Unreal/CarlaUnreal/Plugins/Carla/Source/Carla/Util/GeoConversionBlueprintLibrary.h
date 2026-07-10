// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT/>.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "GeoConversionBlueprintLibrary.generated.h"

/// UE-friendly geographic location (latitude / longitude / altitude).
USTRUCT(BlueprintType)
struct CARLA_API FGeoLocation
{
  GENERATED_BODY()

  /// Latitude in degrees.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GeoLocation")
  double Latitude = 0.0;

  /// Longitude in degrees.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GeoLocation")
  double Longitude = 0.0;

  /// Altitude in metres.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GeoLocation")
  double Altitude = 0.0;

  FGeoLocation() = default;
  FGeoLocation(double InLat, double InLon, double InAlt)
    : Latitude(InLat), Longitude(InLon), Altitude(InAlt) {}
};

/// Blueprint function library for converting between UE world coordinates
/// and geographic (latitude / longitude / altitude) coordinates.
UCLASS()
class CARLA_API UGeoConversionBlueprintLibrary : public UBlueprintFunctionLibrary
{
  GENERATED_BODY()

public:
  /// Convert a UE world location to geographic coordinates (lat/lon/alt).
  /// Uses the GeoProjection from the current map's OpenDRIVE geoReference.
  /// If a LargeMapManager is active, the location is first converted from
  /// local to global before the projection is applied.
  ///
  /// @param WorldContextObject  Any UObject from the current world (auto-supplied by BP).
  /// @param WorldLocation       Location in UE world space (centimetres).
  /// @param OutGeoLocation      Resulting geographic coordinates.
  /// @return True if the conversion succeeded.
  UFUNCTION(BlueprintCallable, Category = "CARLA|GeoConversion",
    meta = (WorldContext = "WorldContextObject"))
  static bool UEToGeoLocation(
      const UObject *WorldContextObject,
      const FVector &WorldLocation,
      FGeoLocation &OutGeoLocation);

  /// Convert geographic coordinates (lat/lon/alt) back to a UE world location.
  ///
  /// @param WorldContextObject  Any UObject from the current world (auto-supplied by BP).
  /// @param GeoLocation         Geographic coordinates (degrees / metres).
  /// @param OutWorldLocation    Resulting UE world location (centimetres).
  /// @return True if the conversion succeeded.
  UFUNCTION(BlueprintCallable, Category = "CARLA|GeoConversion",
    meta = (WorldContext = "WorldContextObject"))
  static bool GeoToUELocation(
      const UObject *WorldContextObject,
      const FGeoLocation &GeoLocation,
      FVector &OutWorldLocation);

  /// Convenience: get the geographic coordinates of any AActor.
  /// Equivalent to calling GetActorLocation() then UEToGeoLocation().
  UFUNCTION(BlueprintCallable, Category = "CARLA|GeoConversion",
    meta = (WorldContext = "WorldContextObject"))
  static bool GetActorGeoLocation(
      const UObject *WorldContextObject,
      const AActor *Actor,
      FGeoLocation &OutGeoLocation);
};
