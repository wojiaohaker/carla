// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/GeoProjection.h"
#include "carla/ros2/publishers/CarlaVehicleDataPublishers.h"
#include <util/enable-ue4-macros.h>

#include "VehicleDataSensor.generated.h"

/// Vehicle data sensor that publishes INS, VehicleState, and ObstacleList
/// messages via ROS2.  Does not stream data back to the Carla client
/// (NoopSerializer).
UCLASS()
class CARLA_API AVehicleDataSensor : public ASensor
{
  GENERATED_BODY()

public:

  AVehicleDataSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription) override;

  void SetOwner(AActor *Owner) override;

  virtual void PostPhysTick(
      UWorld *World, ELevelTick TickType, float DeltaSeconds) override;

  /// Radius (metres) for obstacle detection range.
  void SetDetectionRadius(float Value);
  float GetDetectionRadius() const;

  /// Maximum number of obstacles returned per tick.
  void SetMaxObstacles(int32 Value);
  int32 GetMaxObstacles() const;

protected:

  virtual void BeginPlay() override;

private:

  /// Collect INS fields (position, orientation, velocity, angular velocity).
  void CollectInsData(
      double &out_longitude, double &out_latitude, float &out_altitude,
      float &out_yaw, float &out_pitch, float &out_roll,
      float &out_vx, float &out_vy, float &out_vt, float &out_psd,
      float &out_r, float &out_p, float &out_q,
      uint8_t &out_gps_fix_state, uint8_t &out_satellite_num,
      uint8_t &out_gps_id, uint8_t &out_nav_state,
      uint16_t &out_nav_fault_code, bool &out_is_valid,
      double &out_timestamp);

  /// Collect vehicle-state fields.
  void CollectVehicleStateData(
      uint8_t &out_work_state, uint8_t &out_work_mode,
      uint8_t &out_control_model, uint8_t &out_battery_capacity,
      uint16_t &out_voltage, uint16_t &out_current,
      float &out_speed, float &out_angle, float &out_brake,
      uint16_t &out_fault_code);

  /// Collect obstacle list from actors spawned via CARLA API (ActorRegistry).
  void CollectObstacleListData(
      std::vector<carla::ros2::ObstacleItemData> &out_obstacles);

  /// Geo-projection cached from the episode at BeginPlay.
  carla::geom::GeoProjection CurrentGeoProjection;

  /// Obstacle detection radius in metres.
  float DetectionRadius = 50.0f;

  /// Maximum obstacles per tick.
  int32 MaxObstacles = 20;

  /// Tick accumulator for rate control (50 Hz = 0.02s per tick).
  float TickAccumulator = 0.0f;
  static constexpr float PublishPeriod = 0.02f;  // 50 Hz

};
