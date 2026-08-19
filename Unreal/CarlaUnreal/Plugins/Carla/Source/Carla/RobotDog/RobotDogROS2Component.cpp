// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/RobotDog/RobotDogROS2Component.h"

#include <cmath>

#include "Carla.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"

#ifdef WITH_ROS2
  #include <util/disable-ue4-macros.h>
  #include "carla/ros2/ROS2.h"
  #include <util/enable-ue4-macros.h>
#endif

namespace {
  // UE <-> ROS frame conversion:
  //   UE:   cm, X forward, Y right, Z up, clockwise yaw (degrees)
  //   ROS:  m,  X forward, Y left,  Z up, counter-clockwise yaw (radians)
  constexpr double CmToM = 0.01;
  constexpr double MToCm = 100.0;
}

URobotDogROS2Component::URobotDogROS2Component()
{
  PrimaryComponentTick.bCanEverTick = true;
}

void URobotDogROS2Component::BeginPlay()
{
  Super::BeginPlay();
  InitializePoseFromActor();
}

void URobotDogROS2Component::InitializePoseFromActor()
{
  AActor *Owner = GetOwner();
  if (!Owner)
  {
    return;
  }
  const FVector Location = Owner->GetActorLocation();
  const FRotator Rotation = Owner->GetActorRotation();

  X = Location.X * CmToM;
  Y = -Location.Y * CmToM;
  Z = Location.Z * CmToM;
  Yaw = -FMath::DegreesToRadians(Rotation.Yaw);
  bPoseInitialized = true;
}

void URobotDogROS2Component::ApplyTwist(float Vx, float Vy, float Wz)
{
  CmdVx = Vx;
  CmdVy = Vy;
  CmdWz = Wz;
  TimeSinceLastCmd = 0.0;
}

void URobotDogROS2Component::ApplyPoseToActor()
{
  AActor *Owner = GetOwner();
  if (!Owner)
  {
    return;
  }

  const FVector Location(
      static_cast<float>(X * MToCm),
      static_cast<float>(-Y * MToCm),
      static_cast<float>(Z * MToCm));
  const FRotator Rotation(0.0f, static_cast<float>(-FMath::RadiansToDegrees(Yaw)), 0.0f);

  // Absolute teleport each tick: the integrated odometry stays authoritative
  // regardless of any physics/movement component activity in between.
  Owner->SetActorLocationAndRotation(
      Location, Rotation, /*bSweep=*/false, nullptr, ETeleportType::TeleportPhysics);

  // Feed the world-frame velocity (UE handed, cm/s) into the character
  // movement component so speed-based animation blending keeps playing
  // the walk cycle while the dog moves. Uses the same effective (timeout
  // gated) command that drove the integration this tick.
  if (ACharacter *Character = Cast<ACharacter>(Owner))
  {
    UCharacterMovementComponent *Movement = Character->GetCharacterMovement();
    if (Movement)
    {
      Movement->Velocity = FVector(
          static_cast<float>(LastWorldVx * MToCm),
          static_cast<float>(-LastWorldVy * MToCm),
          0.0f);
    }
  }
}

void URobotDogROS2Component::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction *ThisTickFunction)
{
  Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

#ifdef WITH_ROS2
  if (!carla::ros2::ROS2::GetInstance()->IsEnabled())
  {
    return;
  }
#endif

  if (!bPoseInitialized)
  {
    InitializePoseFromActor();
  }

  const double Dt = static_cast<double>(DeltaTime);

  // Time out stale commands so a stopped Nav2 node leaves the dog still.
  TimeSinceLastCmd += Dt;
  const bool bHasCommand = TimeSinceLastCmd < CmdTimeoutSeconds;
  const float Vx = bHasCommand ? CmdVx : 0.0f;
  const float Vy = bHasCommand ? CmdVy : 0.0f;
  const float Wz = bHasCommand ? CmdWz : 0.0f;

  // Integrate the body-frame twist into the odom frame.
  Yaw += static_cast<double>(Wz) * Dt;
  const double WorldVx = Vx * std::cos(Yaw) - Vy * std::sin(Yaw);
  const double WorldVy = Vx * std::sin(Yaw) + Vy * std::cos(Yaw);
  X += WorldVx * Dt;
  Y += WorldVy * Dt;
  LastWorldVx = WorldVx;
  LastWorldVy = WorldVy;

  ApplyPoseToActor();

#ifdef WITH_ROS2
  // Publish /odom + TF (odom -> base_link, base_link -> laser_up) at 50 Hz.
  OdomPublishAccumulator += Dt;
  if (OdomPublishAccumulator >= OdomPublishPeriod)
  {
    OdomPublishAccumulator = 0.0;
    carla::ros2::ROS2::GetInstance()->PublishRobotDogOdometry(
        X, Y, Z, Yaw,
        static_cast<double>(Vx), static_cast<double>(Vy), static_cast<double>(Wz));
  }
#endif
}
