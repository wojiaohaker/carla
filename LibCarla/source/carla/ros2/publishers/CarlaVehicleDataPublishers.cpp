// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT/>.

#include "carla/ros2/publishers/CarlaVehicleDataPublishers.h"

#include "carla/Logging.h"
#include "carla/ros2/publishers/PublisherImpl.h"
#include "carla/ros2/types/InsData.h"
#include "carla/ros2/types/InsDataPubSubTypes.h"
#include "carla/ros2/types/VehicleState.h"
#include "carla/ros2/types/VehicleStatePubSubTypes.h"
#include "carla/ros2/types/ObstacleList.h"
#include "carla/ros2/types/ObstacleListPubSubTypes.h"

namespace carla {
namespace ros2 {

// ===========================================================================
// Traits
// ===========================================================================
struct CarlaInsDataMsgTraits {
  using msg_type = vehicle_msgs::msg::InsData;
  using msg_pubsub_type = vehicle_msgs::msg::InsDataPubSubType;
};

struct CarlaVehicleStateMsgTraits {
  using msg_type = vehicle_msgs::msg::VehicleState;
  using msg_pubsub_type = vehicle_msgs::msg::VehicleStatePubSubType;
};

struct CarlaObstacleListMsgTraits {
  using msg_type = vehicle_msgs::msg::ObstacleList;
  using msg_pubsub_type = vehicle_msgs::msg::ObstacleListPubSubType;
};

// ===========================================================================
// CarlaInsDataPublisher
// ===========================================================================
CarlaInsDataPublisher::CarlaInsDataPublisher(
    std::string base_topic_name, std::string frame_id)
  : BasePublisher(std::move(base_topic_name), std::move(frame_id)),
    _impl(std::make_shared<PublisherImpl<CarlaInsDataMsgTraits>>()) {
  if (!_impl->Init(GetBaseTopicName())) {
    log_error("CarlaInsDataPublisher: failed to initialise writer for", GetBaseTopicName());
  }
}

CarlaInsDataPublisher::~CarlaInsDataPublisher() = default;

bool CarlaInsDataPublisher::Publish() { return _impl->Publish(); }

bool CarlaInsDataPublisher::Write(
    std::int32_t seconds, std::uint32_t nanoseconds,
    double longitude, double latitude, float altitude,
    float yaw, float pitch, float roll,
    float vx, float vy, float vt, float psd,
    float r, float p, float q,
    uint8_t gps_fix_state, uint8_t satellite_num, uint8_t gps_id,
    uint8_t nav_state, uint16_t nav_fault_code,
    bool is_valid, double timestamp) {
  auto *msg = _impl->GetMessage();
  msg->header().stamp().sec(seconds);
  msg->header().stamp().nanosec(nanoseconds);
  msg->header().frame_id(GetFrameId());
  msg->longitude(longitude);
  msg->latitude(latitude);
  msg->altitude(altitude);
  msg->yaw(yaw);
  msg->pitch(pitch);
  msg->roll(roll);
  msg->vx(vx);
  msg->vy(vy);
  msg->vt(vt);
  msg->psd(psd);
  msg->r(r);
  msg->p(p);
  msg->q(q);
  msg->gps_fix_state(gps_fix_state);
  msg->satellite_num(satellite_num);
  msg->gps_id(gps_id);
  msg->nav_state(nav_state);
  msg->nav_fault_code(nav_fault_code);
  msg->is_valid(is_valid);
  msg->timestamp(timestamp);
  return true;
}

// ===========================================================================
// CarlaVehicleStatePublisher
// ===========================================================================
CarlaVehicleStatePublisher::CarlaVehicleStatePublisher(
    std::string base_topic_name, std::string frame_id)
  : BasePublisher(std::move(base_topic_name), std::move(frame_id)),
    _impl(std::make_shared<PublisherImpl<CarlaVehicleStateMsgTraits>>()) {
  if (!_impl->Init(GetBaseTopicName())) {
    log_error("CarlaVehicleStatePublisher: failed to initialise writer for", GetBaseTopicName());
  }
}

CarlaVehicleStatePublisher::~CarlaVehicleStatePublisher() = default;

bool CarlaVehicleStatePublisher::Publish() { return _impl->Publish(); }

bool CarlaVehicleStatePublisher::Write(
    std::int32_t seconds, std::uint32_t nanoseconds,
    uint8_t work_state, uint8_t work_mode, uint8_t control_model,
    uint8_t battery_capacity, uint16_t voltage, uint16_t current,
    float speed, float angle, float brake, uint16_t fault_code) {
  auto *msg = _impl->GetMessage();
  msg->header().stamp().sec(seconds);
  msg->header().stamp().nanosec(nanoseconds);
  msg->header().frame_id(GetFrameId());
  msg->work_state(work_state);
  msg->work_mode(work_mode);
  msg->control_model(control_model);
  msg->battery_capacity(battery_capacity);
  msg->voltage(voltage);
  msg->current(current);
  msg->speed(speed);
  msg->angle(angle);
  msg->brake(brake);
  msg->fault_code(fault_code);
  return true;
}

// ===========================================================================
// CarlaObstacleListPublisher
// ===========================================================================
CarlaObstacleListPublisher::CarlaObstacleListPublisher(
    std::string base_topic_name, std::string frame_id)
  : BasePublisher(std::move(base_topic_name), std::move(frame_id)),
    _impl(std::make_shared<PublisherImpl<CarlaObstacleListMsgTraits>>()) {
  if (!_impl->Init(GetBaseTopicName())) {
    log_error("CarlaObstacleListPublisher: failed to initialise writer for", GetBaseTopicName());
  }
}

CarlaObstacleListPublisher::~CarlaObstacleListPublisher() = default;

bool CarlaObstacleListPublisher::Publish() { return _impl->Publish(); }

bool CarlaObstacleListPublisher::Write(
    std::int32_t seconds, std::uint32_t nanoseconds,
    const std::vector<ObstacleItemData>& obstacles) {
  auto *msg = _impl->GetMessage();
  msg->header().stamp().sec(seconds);
  msg->header().stamp().nanosec(nanoseconds);
  msg->header().frame_id(GetFrameId());

  std::vector<vehicle_msgs::msg::ObstacleItem> items;
  items.reserve(obstacles.size());
  for (const auto& obs : obstacles) {
    vehicle_msgs::msg::ObstacleItem item;
    item.id(obs.id);
    item.type(obs.type);
    item.x(obs.x);
    item.y(obs.y);
    item.length(obs.length);
    item.width(obs.width);
    item.height(obs.height);
    item.course(obs.course);
    item.speed(obs.speed);
    items.push_back(std::move(item));
  }
  msg->obstacles(std::move(items));
  return true;
}

}  // namespace ros2
}  // namespace carla
