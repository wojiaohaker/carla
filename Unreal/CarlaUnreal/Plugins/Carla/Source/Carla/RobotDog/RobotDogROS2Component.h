// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RobotDogROS2Component.generated.h"

/// Drives a robot dog actor (e.g. BP_ThirdPersonCharacter) from ROS 2
/// /cmd_vel commands for the Gazebo -> CarlaUnreal navigation migration.
///
/// The component keeps an authoritative pose integrated in the ROS odom
/// frame (meters, ROS handedness: X forward, Y left, Z up, CCW yaw) and
/// applies it to the owner actor every tick with a physics teleport. The
/// integrated pose is fed back to the ROS 2 layer at 50 Hz as /odom plus
/// the odom -> base_link TF (see carla::ros2::ROS2::PublishRobotDogOdometry).
/// For ACharacter owners the CharacterMovementComponent velocity is also
/// set so speed-based animation blending keeps working.
UCLASS(ClassGroup = (Carla), meta = (BlueprintSpawnableComponent))
class CARLA_API URobotDogROS2Component : public UActorComponent
{
  GENERATED_BODY()

public:
  URobotDogROS2Component();

  virtual void BeginPlay() override;
  virtual void TickComponent(
      float DeltaTime,
      ELevelTick TickType,
      FActorComponentTickFunction *ThisTickFunction) override;

  /// Called by the ROS 2 callback path (game thread) whenever a new
  /// /cmd_vel message arrives. Velocities follow ROS conventions:
  /// linear in m/s (x forward, y left), angular.z in rad/s (CCW positive).
  void ApplyTwist(float Vx, float Vy, float Wz);

private:
  /// Seeds the integrated pose from the actor spawn transform.
  void InitializePoseFromActor();

  /// Applies the authoritative ROS pose to the owner actor.
  void ApplyPoseToActor();

  // Authoritative pose in the ROS odom frame.
  double X{0.0};
  double Y{0.0};
  double Z{0.0};
  double Yaw{0.0};

  // Latest /cmd_vel command (body frame).
  float CmdVx{0.0f};
  float CmdVy{0.0f};
  float CmdWz{0.0f};

  // Seconds since the last /cmd_vel message; the command times out to
  // zero so a dead Nav2 node cannot keep the dog drifting.
  double TimeSinceLastCmd{0.0};
  static constexpr double CmdTimeoutSeconds = 0.5;

  // Odometry publish rate limiter (50 Hz, matching the migration spec).
  double OdomPublishAccumulator{0.0};
  static constexpr double OdomPublishPeriod = 1.0 / 50.0;

  // Last integrated world-frame velocity (ROS handed, m/s); forwarded to
  // the character movement component for animation blending.
  double LastWorldVx{0.0};
  double LastWorldVy{0.0};

  bool bPoseInitialized{false};
};
