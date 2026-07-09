// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT/>.

#pragma once

#include <cstdint>

namespace carla {
namespace ros2 {

/// Obstacle item data passed from UE side to the publisher layer.
/// Defined separately so that both ROS2.h and the publishers can share it
/// without pulling in heavy includes.
struct ObstacleItemData {
  int32_t id = 0;
  uint8_t type = 0;
  double x = 0.0;
  double y = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  double course = 0.0;
  double speed = 0.0;
};

}  // namespace ros2
}  // namespace carla
