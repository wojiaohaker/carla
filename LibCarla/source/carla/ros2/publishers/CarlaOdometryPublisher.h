// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/publishers/BasePublisher.h"

#include <cstdint>
#include <memory>
#include <string>

namespace carla {
namespace ros2 {

template <typename Traits> class PublisherImpl;
struct CarlaOdometryMsgTraits;

// Publishes nav_msgs::msg::Odometry for the robot dog on the fixed "rt/odom"
// topic. Pose is expressed in the "odom" frame (meters, ROS handedness),
// twist in the body frame ("base_link"). The Nav2 stack and AMCL consume
// this topic directly, mirroring what CHAMP published in the original
// Gazebo setup.
class CarlaOdometryPublisher : public BasePublisher {
public:
  CarlaOdometryPublisher(std::string base_topic_name, std::string frame_id);
  ~CarlaOdometryPublisher() override;

  bool Publish() override;

  // Stages an Odometry message for the next Publish().
  //   x, y, z      : position in the odom frame [m]
  //   qw..qz       : orientation quaternion (ROS handedness)
  //   vx, vy, vz   : body-frame linear velocity [m/s]
  //   wx, wy, wz   : body-frame angular velocity [rad/s]
  bool Write(
      std::int32_t seconds,
      std::uint32_t nanoseconds,
      const std::string &child_frame_id,
      double x, double y, double z,
      double qw, double qx, double qy, double qz,
      double vx, double vy, double vz,
      double wx, double wy, double wz);

private:
  std::shared_ptr<PublisherImpl<CarlaOdometryMsgTraits>> _impl;
};

}  // namespace ros2
}  // namespace carla
