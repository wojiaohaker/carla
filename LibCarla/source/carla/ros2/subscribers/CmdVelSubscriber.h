// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <string>

#include "carla/ros2/ROS2CallbackData.h"
#include "carla/ros2/subscribers/BaseSubscriber.h"

namespace carla {
namespace ros2 {

// Forward declarations keep the FastDDS-heavy SubscriberImpl<> definition out of the
// main carla-server compile unit. The full instantiation lives in
// CmdVelSubscriber.cpp, which is built by the carla-ros2-native
// ExternalProject (where FastDDS headers are on the include path).
template <typename Traits> class SubscriberImpl;
struct CmdVelTraits;

// Subscribes to the Nav2 velocity command topic /cmd_vel
// (geometry_msgs/msg/Twist) and forwards it to the robot dog actor as a
// TwistControl. The topic is fixed ("rt/cmd_vel") to match the ROS 2
// navigation stack topic naming, unlike vehicle control topics which are
// namespaced per actor.
class CmdVelSubscriber : public BaseSubscriber {
public:
  CmdVelSubscriber(void *vehicle, std::string topic_name, std::string frame_id);
  ~CmdVelSubscriber() override;

  void ProcessMessages(ActorCallback callback) override;

protected:
  ROS2CallbackData GetMessage() override;

private:
  std::shared_ptr<SubscriberImpl<CmdVelTraits>> _impl;
};

}  // namespace ros2
}  // namespace carla
