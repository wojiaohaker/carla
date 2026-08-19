// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/subscribers/CmdVelSubscriber.h"

#include "carla/Logging.h"
#include "carla/ros2/ROS2CallbackData.h"
#include "carla/ros2/subscribers/SubscriberImpl.h"
#include "carla/ros2/types/Twist.h"
#include "carla/ros2/types/TwistPubSubTypes.h"

namespace carla {
namespace ros2 {

struct CmdVelTraits {
  using msg_type = geometry_msgs::msg::Twist;
  using msg_pubsub_type = geometry_msgs::msg::TwistPubSubType;
};

CmdVelSubscriber::CmdVelSubscriber(
    void *vehicle, std::string topic_name, std::string frame_id)
  : BaseSubscriber(vehicle, topic_name, std::move(frame_id)),
    _impl(std::make_shared<SubscriberImpl<CmdVelTraits>>()) {
  if (!_impl->Init(this->GetBaseTopicName())) {
    log_error("CmdVelSubscriber failed to initialize on topic",
              this->GetBaseTopicName());
  }
}

CmdVelSubscriber::~CmdVelSubscriber() = default;

ROS2CallbackData CmdVelSubscriber::GetMessage() {
  auto message = _impl->GetMessage();

  TwistControl twist;
  twist.vx = static_cast<float>(message.linear().x());
  twist.vy = static_cast<float>(message.linear().y());
  twist.wz = static_cast<float>(message.angular().z());
  return twist;
}

void CmdVelSubscriber::ProcessMessages(ActorCallback callback) {
  if (_impl->HasNewMessage()) {
    auto twist = this->GetMessage();
    callback(this->GetActor(), twist);
  }
}

}  // namespace ros2
}  // namespace carla
