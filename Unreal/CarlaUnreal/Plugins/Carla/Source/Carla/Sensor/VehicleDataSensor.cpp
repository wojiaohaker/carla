// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Sensor/VehicleDataSensor.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorRegistry.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/MapGen/LargeMapManager.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Vector3D.h"
#include "carla/ros2/ROS2.h"
#include <util/enable-ue4-macros.h>

#include "ChaosWheeledVehicleMovementComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/Pawn.h"

#include <cmath>

// ---------------------------------------------------------------------------
// Construction / definition
// ---------------------------------------------------------------------------

AVehicleDataSensor::AVehicleDataSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
  PrimaryActorTick.TickGroup = TG_PostPhysics;
}

FActorDefinition AVehicleDataSensor::GetSensorDefinition()
{
  FActorDefinition Definition;
  UActorBlueprintFunctionLibrary::MakeVehicleDataDefinition(Definition);
  return Definition;
}

void AVehicleDataSensor::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  DetectionRadius = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToFloat(
      "detection_radius", ActorDescription.Variations, DetectionRadius);
  MaxObstacles = UActorBlueprintFunctionLibrary::RetrieveActorAttributeToInt(
      "max_obstacles", ActorDescription.Variations, MaxObstacles);
}

void AVehicleDataSensor::SetOwner(AActor *OwningActor)
{
  Super::SetOwner(OwningActor);
}

void AVehicleDataSensor::BeginPlay()
{
  Super::BeginPlay();

  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if (Episode)
  {
    CurrentGeoProjection = Episode->GetGeoProjection();
  }
}

// ---------------------------------------------------------------------------
// PostPhysTick – main data-collection entry point
// ---------------------------------------------------------------------------

void AVehicleDataSensor::PostPhysTick(
    UWorld *World, ELevelTick TickType, float DeltaSeconds)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(AVehicleDataSensor::PostPhysTick);

#if defined(WITH_ROS2)
  auto ROS2 = carla::ros2::ROS2::GetInstance();
  if (!ROS2->IsEnabled())
  {
    return;
  }

  // Rate control: accumulate delta time and only publish at 50 Hz
  TickAccumulator += DeltaSeconds;
  if (TickAccumulator < PublishPeriod)
  {
    return;  // Not enough time elapsed, skip this tick
  }
  TickAccumulator -= PublishPeriod;  // Subtract period (not reset to 0) to avoid drift

  // --- INS data ---
  double longitude = 0.0, latitude = 0.0;
  float altitude = 0.0f;
  float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
  float vx = 0.0f, vy = 0.0f, vt = 0.0f, psd = 0.0f;
  float r = 0.0f, p = 0.0f, q = 0.0f;
  uint8_t gps_fix_state = 0, satellite_num = 0, gps_id = 0, nav_state = 0;
  uint16_t nav_fault_code = 0;
  bool is_valid = false;
  double timestamp = 0.0;
  CollectInsData(longitude, latitude, altitude,
                 yaw, pitch, roll,
                 vx, vy, vt, psd,
                 r, p, q,
                 gps_fix_state, satellite_num, gps_id,
                 nav_state, nav_fault_code, is_valid, timestamp);

  // --- VehicleState data ---
  uint8_t work_state = 0, work_mode = 0, control_model = 0;
  uint8_t battery_capacity = 100;
  uint16_t voltage = 0, current = 0;
  float speed = 0.0f, angle = 0.0f, brake = 0.0f;
  uint16_t fault_code = 0;
  CollectVehicleStateData(work_state, work_mode, control_model,
                          battery_capacity, voltage, current,
                          speed, angle, brake, fault_code);

  // --- ObstacleList data ---
  std::vector<carla::ros2::ObstacleItemData> obstacles;
  CollectObstacleListData(obstacles);

  // --- Publish ---
  auto StreamId = carla::streaming::detail::token_type(GetToken()).get_stream_id();
  auto DataStream = GetDataStream(*this);

  AActor *ParentActor = GetAttachParentActor();
  carla::geom::Transform SensorTransform;
  if (ParentActor)
  {
    FTransform LocalTransform = GetActorTransform().GetRelativeTransform(
        ParentActor->GetActorTransform());
    SensorTransform = LocalTransform;
  }
  else
  {
    SensorTransform = DataStream.GetSensorTransform();
  }

  ROS2->ProcessDataFromVehicleData(
      DataStream.GetSensorType(), StreamId, SensorTransform,
      // INS
      longitude, latitude, altitude,
      yaw, pitch, roll,
      vx, vy, vt, psd,
      r, p, q,
      gps_fix_state, satellite_num, gps_id,
      nav_state, nav_fault_code,
      is_valid, timestamp,
      // VehicleState
      work_state, work_mode, control_model,
      battery_capacity, voltage, current,
      speed, angle, brake, fault_code,
      // ObstacleList
      obstacles,
      this);
#endif  // WITH_ROS2
}

// ---------------------------------------------------------------------------
// INS data collection
// ---------------------------------------------------------------------------

void AVehicleDataSensor::CollectInsData(
    double &out_longitude, double &out_latitude, float &out_altitude,
    float &out_yaw, float &out_pitch, float &out_roll,
    float &out_vx, float &out_vy, float &out_vt, float &out_psd,
    float &out_r, float &out_p, float &out_q,
    uint8_t &out_gps_fix_state, uint8_t &out_satellite_num,
    uint8_t &out_gps_id, uint8_t &out_nav_state,
    uint16_t &out_nav_fault_code, bool &out_is_valid,
    double &out_timestamp)
{
  // --- Position (convert UE location -> geo) ---
  FVector ActorLocation = GetActorLocation();
  ALargeMapManager *LargeMap = UCarlaStatics::GetLargeMapManager(GetWorld());
  if (LargeMap)
  {
    ActorLocation = LargeMap->LocalToGlobalLocation(ActorLocation);
  }
  carla::geom::Location Location = ActorLocation;
  carla::geom::GeoLocation GeoLoc = CurrentGeoProjection.TransformToGeoLocation(Location);

  out_longitude = GeoLoc.longitude;
  out_latitude = GeoLoc.latitude;
  out_altitude = static_cast<float>(GeoLoc.altitude);

  // --- Orientation (degrees -> radians) ---
  /*const FRotator Rot = GetActorRotation();
  constexpr float DEG_TO_RAD = PI / 180.0f;
  out_yaw   = Rot.Yaw   * DEG_TO_RAD;
  out_pitch  = Rot.Pitch  * DEG_TO_RAD;
  out_roll   = Rot.Roll   * DEG_TO_RAD;*/
  
  const FRotator Rot = GetActorRotation();
  out_yaw   = Rot.Yaw;
  out_pitch  = Rot.Pitch;
  out_roll   = Rot.Roll;

  // --- Velocity in body frame ---
  AActor *OwnerActor = GetOwner();
  if (OwnerActor)
  {
    // UE velocity is in cm/s in world frame; convert to m/s in body frame.
    const FVector WorldVelocityCmS = OwnerActor->GetVelocity();
    const FVector WorldVelocityMs = WorldVelocityCmS * 0.01;  // cm/s -> m/s

    const FQuat ActorQuat = GetActorQuat();
    const FVector BodyVelocity = ActorQuat.UnrotateVector(WorldVelocityMs);

    out_vx = static_cast<float>(BodyVelocity.X);  // forward
    out_vy = static_cast<float>(BodyVelocity.Y);  // right
    out_vt = static_cast<float>(WorldVelocityMs.Size());  // total speed
  }

  // --- PSD (position standard deviation) – default 0, no noise model ---
  out_psd = 0.0f;

  // --- Angular velocity (rad/s) in body frame ---
  const auto *RootComp = Cast<UPrimitiveComponent>(GetRootComponent());
  if (RootComp)
  {
    const FQuat ActorGlobalRotation = RootComp->GetComponentTransform().GetRotation();
    const FVector GlobalAngularVelocity = RootComp->GetPhysicsAngularVelocityInRadians();
    const FVector LocalAngularVelocity = ActorGlobalRotation.UnrotateVector(GlobalAngularVelocity);
    out_r = static_cast<float>(LocalAngularVelocity.X);
    out_p = static_cast<float>(LocalAngularVelocity.Y);
    out_q = static_cast<float>(LocalAngularVelocity.Z);
  }
  else
  {
    out_r = out_p = out_q = 0.0f;
  }

  // --- GPS status defaults (simulated = perfect fix) ---
  out_gps_fix_state = 1;   // 1 = fix
  out_satellite_num = 12;
  out_gps_id = 0;
  out_nav_state = 0;
  out_nav_fault_code = 0;
  out_is_valid = true;

  // --- Timestamp (seconds since simulation start) ---
  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(GetWorld());
  if (Episode)
  {
    out_timestamp = Episode->GetElapsedGameTime();
  }
}

// ---------------------------------------------------------------------------
// VehicleState data collection
// ---------------------------------------------------------------------------

void AVehicleDataSensor::CollectVehicleStateData(
    uint8_t &out_work_state, uint8_t &out_work_mode,
    uint8_t &out_control_model, uint8_t &out_battery_capacity,
    uint16_t &out_voltage, uint16_t &out_current,
    float &out_speed, float &out_angle, float &out_brake,
    uint16_t &out_fault_code)
{
  // Defaults for fields not directly available from the simulation.
  out_work_state = 1;        // 1 = running
  out_work_mode = 0;         // 0 = auto
  out_control_model = 0;     // 0 = default
  out_battery_capacity = 100;
  out_voltage = 0;
  out_current = 0;
  out_fault_code = 0;

  AActor *OwnerActor = GetOwner();
  if (!OwnerActor)
  {
    out_speed = out_angle = out_brake = 0.0f;
    return;
  }

  // Speed in m/s (convention: message stores m/s * 10).
  auto *CarlaVehicle = Cast<ACarlaWheeledVehicle>(OwnerActor);
  if (CarlaVehicle)
  {
    const float ForwardSpeedMs = CarlaVehicle->GetVehicleForwardSpeed() * 0.01f;  // cm/s -> m/s
    out_speed = ForwardSpeedMs * 10.0f;  // scale by 10 per message convention
  }
  else
  {
    const FVector VelMs = OwnerActor->GetVelocity() * 0.01f;
    out_speed = static_cast<float>(VelMs.Size()) * 10.0f;
  }

  // Steering angle and brake from the Chaos movement component.
  auto *MovementComp = Cast<UChaosWheeledVehicleMovementComponent>(
      OwnerActor->FindComponentByClass<UChaosWheeledVehicleMovementComponent>());
  if (MovementComp)
  {
    // SteeringInput is in [-1, 1]; convert to approximate degrees.
    out_angle = MovementComp->GetSteeringInput() * 30.0f;  // ~max 30 deg
    out_brake = MovementComp->GetBrakeInput();
  }
  else
  {
    out_angle = 0.0f;
    out_brake = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// ObstacleList data collection
// ---------------------------------------------------------------------------

void AVehicleDataSensor::CollectObstacleListData(
    std::vector<carla::ros2::ObstacleItemData> &out_obstacles)
{
  out_obstacles.clear();

  UWorld *CurrentWorld = GetWorld();
  if (!CurrentWorld)
  {
    return;
  }

  // Get the ActorRegistry — only contains actors spawned via CARLA API
  // (vehicles, walkers, props). Static map objects are NOT in the registry.
  const UCarlaEpisode *Episode = UCarlaStatics::GetCurrentEpisode(CurrentWorld);
  if (!Episode)
  {
    return;
  }
  const FActorRegistry &Registry = Episode->GetActorRegistry();
  if (Registry.IsEmpty())
  {
    UE_LOG(LogTemp, Warning, TEXT("CollectObstacleListData: ActorRegistry is empty"));
    return;
  }

  UE_LOG(LogTemp, Log, TEXT("CollectObstacleListData: Registry has %d actors"), Registry.Num());

  // Vehicle pose for world -> body conversion.
  const FVector VehiclePos = GetActorLocation();
  const FQuat  VehicleQuat = GetActorQuat();
  const float  RadiusCm    = DetectionRadius * 100.0f;  // metres -> cm
  const float  RadiusCmSq  = RadiusCm * RadiusCm;

  AActor *OwnerActor = GetOwner();

  int32 ObstacleId = 0;
  for (const auto &Pair : Registry)
  {
    FCarlaActor *CarlaActor = Pair.Value.Get();
    if (!CarlaActor || !CarlaActor->IsAlive())
    {
      continue;
    }

    const auto ActorType = CarlaActor->GetActorType();
    const FActorInfo *ActorInfo = CarlaActor->GetActorInfo();
    FString DescId = ActorInfo ? ActorInfo->Description.Id : TEXT("(null)");
    FString RoleNameVal = TEXT("(none)");
    if (ActorInfo)
    {
      const FActorAttribute *RN = ActorInfo->Description.Variations.Find("role_name");
      if (RN) RoleNameVal = RN->Value;
    }
    AActor *Actor = CarlaActor->GetActor();
    FVector ActorLoc = Actor ? Actor->GetActorLocation() : FVector::ZeroVector;

    UE_LOG(LogTemp, Log, TEXT("  [Registry] Id=%d, Type=%d, DescId=%s, role_name=%s, Loc=(%.0f,%.0f,%.0f)"),
        Pair.Key, (int32)ActorType, *DescId, *RoleNameVal, ActorLoc.X, ActorLoc.Y, ActorLoc.Z);

    if (ObstacleId >= MaxObstacles)
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: MaxObstacles reached (%d)"), MaxObstacles);
      break;
    }

    // Only process Vehicle, Walker, Other (props). Skip sensors/traffic.
    if (ActorType != FCarlaActor::ActorType::Vehicle &&
        ActorType != FCarlaActor::ActorType::Walker &&
        ActorType != FCarlaActor::ActorType::Other)
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: wrong ActorType"));
      continue;
    }

    // Check role_name.
    if (!ActorInfo)
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: no ActorInfo"));
      continue;
    }
    const FActorAttribute *RoleName = ActorInfo->Description.Variations.Find("role_name");
    if (!RoleName || RoleName->Value != TEXT("obstacle"))
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: role_name is not 'obstacle'"));
      continue;
    }

    if (!Actor || Actor == this || Actor == OwnerActor)
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: self or owner"));
      continue;
    }

    // Real-time actor location.
    const FVector ObjLoc = Actor->GetActorLocation();

    // Distance filter (cm).
    const FVector Delta = ObjLoc - VehiclePos;
    if (Delta.SizeSquared() > RadiusCmSq)
    {
      UE_LOG(LogTemp, Log, TEXT("    -> skipped: out of range (dist=%.0f cm, radius=%.0f cm)"),
          Delta.Size(), RadiusCm);
      continue;
    }

    UE_LOG(LogTemp, Log, TEXT("    -> OBSTACLE FOUND! dist=%.0f cm"), Delta.Size());

    // Convert to vehicle body frame (cm -> m).
    // UE body: X=forward, Y=right, Z=down.
    const FVector RelativeBody = VehicleQuat.UnrotateVector(Delta) * 0.01f;

    // Bounding box from ActorInfo (cm -> m, full extent).
    FVector BBExtent = ActorInfo->BoundingBox.Extent * 0.01f * 2.0f;

    // Map to algorithm convention:
    //   x = lateral offset (right positive) = Y_body
    //   y = forward distance (front positive) = X_body
    const double MsgX = static_cast<double>( RelativeBody.Y);
    const double MsgY = static_cast<double>( RelativeBody.X);

    // Course in degrees: angle from forward axis (y-axis in algorithm convention).
    // atan2(lateral, forward) = atan2(x, y)
    const double CourseDeg = FMath::RadiansToDegrees(
        FMath::Atan2(MsgX, MsgY));

    // Determine obstacle type and speed from CarlaActor type.
    uint8_t ObsType = 3;  // default: static obstacle (prop)
    double ObsSpeed = 0.0;
    switch (ActorType)
    {
      case FCarlaActor::ActorType::Vehicle:
        ObsType = 1;  // vehicle
        ObsSpeed = CarlaActor->GetActorVelocity().Size() * 0.01f;  // cm/s -> m/s
        break;
      case FCarlaActor::ActorType::Walker:
        ObsType = 2;  // pedestrian
        ObsSpeed = CarlaActor->GetActorVelocity().Size() * 0.01f;  // cm/s -> m/s
        break;
      default:
        ObsType = 3;  // static obstacle (prop)
        break;
    }

    carla::ros2::ObstacleItemData item;
    item.id     = ObstacleId++;
    item.type   = ObsType;
    item.x      = MsgX;
    item.y      = MsgY;
    item.length = static_cast<double>(BBExtent.X);
    item.width  = static_cast<double>(BBExtent.Y);
    item.height = static_cast<double>(BBExtent.Z);
    item.course = CourseDeg;
    item.speed  = ObsSpeed;
    out_obstacles.push_back(item);
  }

  UE_LOG(LogTemp, Log, TEXT("CollectObstacleListData: collected %d obstacles"), ObstacleId);
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

void AVehicleDataSensor::SetDetectionRadius(float Value)
{
  DetectionRadius = FMath::Max(0.0f, Value);
}

float AVehicleDataSensor::GetDetectionRadius() const
{
  return DetectionRadius;
}

void AVehicleDataSensor::SetMaxObstacles(int32 Value)
{
  MaxObstacles = FMath::Max(0, Value);
}

int32 AVehicleDataSensor::GetMaxObstacles() const
{
  return MaxObstacles;
}
