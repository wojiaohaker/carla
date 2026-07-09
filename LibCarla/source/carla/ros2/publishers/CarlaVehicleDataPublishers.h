// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT/>.

#pragma once

#include "carla/ros2/publishers/BasePublisher.h"
#include "carla/ros2/types/ObstacleItemData.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace carla {
namespace ros2 {

template <typename Traits> class PublisherImpl;
struct CarlaInsDataMsgTraits;
struct CarlaVehicleStateMsgTraits;
struct CarlaObstacleListMsgTraits;

// Publishes vehicle_msgs::msg::InsData.
class CarlaInsDataPublisher : public BasePublisher {
public:
  CarlaInsDataPublisher(std::string base_topic_name, std::string frame_id);
  ~CarlaInsDataPublisher() override;

  CarlaInsDataPublisher(const CarlaInsDataPublisher &) = delete;
  CarlaInsDataPublisher &operator=(const CarlaInsDataPublisher &) = delete;
  CarlaInsDataPublisher(CarlaInsDataPublisher &&) noexcept = delete;
  CarlaInsDataPublisher &operator=(CarlaInsDataPublisher &&) noexcept = delete;

  bool Publish() override;
  bool Write(
      std::int32_t seconds, std::uint32_t nanoseconds,
      double longitude, double latitude, float altitude,
      float yaw, float pitch, float roll,
      float vx, float vy, float vt, float psd,
      float r, float p, float q,
      uint8_t gps_fix_state, uint8_t satellite_num, uint8_t gps_id,
      uint8_t nav_state, uint16_t nav_fault_code,
      bool is_valid, double timestamp);

private:
  std::shared_ptr<PublisherImpl<CarlaInsDataMsgTraits>> _impl;
};

// Publishes vehicle_msgs::msg::VehicleState.
class CarlaVehicleStatePublisher : public BasePublisher {
public:
  CarlaVehicleStatePublisher(std::string base_topic_name, std::string frame_id);
  ~CarlaVehicleStatePublisher() override;

  CarlaVehicleStatePublisher(const CarlaVehicleStatePublisher &) = delete;
  CarlaVehicleStatePublisher &operator=(const CarlaVehicleStatePublisher &) = delete;
  CarlaVehicleStatePublisher(CarlaVehicleStatePublisher &&) noexcept = delete;
  CarlaVehicleStatePublisher &operator=(CarlaVehicleStatePublisher &&) noexcept = delete;

  bool Publish() override;
  bool Write(
      std::int32_t seconds, std::uint32_t nanoseconds,
      uint8_t work_state, uint8_t work_mode, uint8_t control_model,
      uint8_t battery_capacity, uint16_t voltage, uint16_t current,
      float speed, float angle, float brake, uint16_t fault_code);

private:
  std::shared_ptr<PublisherImpl<CarlaVehicleStateMsgTraits>> _impl;
};

// Publishes vehicle_msgs::msg::ObstacleList.
class CarlaObstacleListPublisher : public BasePublisher {
public:
  CarlaObstacleListPublisher(std::string base_topic_name, std::string frame_id);
  ~CarlaObstacleListPublisher() override;

  CarlaObstacleListPublisher(const CarlaObstacleListPublisher &) = delete;
  CarlaObstacleListPublisher &operator=(const CarlaObstacleListPublisher &) = delete;
  CarlaObstacleListPublisher(CarlaObstacleListPublisher &&) noexcept = delete;
  CarlaObstacleListPublisher &operator=(CarlaObstacleListPublisher &&) noexcept = delete;

  bool Publish() override;
  bool Write(
      std::int32_t seconds, std::uint32_t nanoseconds,
      const std::vector<ObstacleItemData>& obstacles);

private:
  std::shared_ptr<PublisherImpl<CarlaObstacleListMsgTraits>> _impl;
};

}  // namespace ros2
}  // namespace carla
