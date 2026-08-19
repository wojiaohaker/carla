// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <cstdint>
#include <functional>
#include <variant>

namespace carla {
namespace ros2 {

  struct VehicleControl
  {
    float   throttle;
    float   steer;
    float   brake;
    bool    hand_brake;
    bool    reverse;
    int32_t gear;
    bool    manual_gear_shift;
  };

  struct AckermannControl
  {
    float steer;
    float steer_speed;
    float speed;
    float acceleration;
    float jerk;
  };

  struct MessageControl
  {
    const char* message;
  };

  // Velocity command parsed from geometry_msgs/msg/Twist on /cmd_vel
  // (robot dog). Linear velocities in m/s (x forward, y left), angular.z
  // in rad/s (counter-clockwise positive) — ROS convention.
  struct TwistControl
  {
    float vx;
    float vy;
    float wz;
  };

  using ROS2CallbackData = std::variant<VehicleControl, AckermannControl, TwistControl>;
  using ROS2MessageCallbackData = std::variant<MessageControl>;

  using ActorCallback = std::function<void(void *actor, ROS2CallbackData data)>;
  using ActorMessageCallback = std::function<void(void *actor, ROS2MessageCallbackData data)>;

} // namespace ros2
} // namespace carla
