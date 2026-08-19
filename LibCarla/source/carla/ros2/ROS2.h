// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Buffer.h"
#include "carla/BufferView.h"
#include "carla/geom/Transform.h"
#include "carla/ros2/ROS2CallbackData.h"
#include "carla/streaming/detail/Types.h"

#include "carla/ros2/types/ObstacleItemData.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>

// forward declarations
class AActor;
namespace carla {
  namespace geom {
    class GeoLocation;
    struct Vector3D;
  }
  namespace sensor {
    namespace data {
      struct DVSEvent;
      class LidarData;
      class SemanticLidarData;
      class RadarData;
    }
  }
}

namespace carla {
namespace ros2 {

class BasePublisher;
class BaseSubscriber;
class CarlaCameraPublisher;
class CarlaClockPublisher;
class CarlaTransformPublisher;
class CarlaOdometryPublisher;
class CarlaInsDataPublisher;
class CarlaVehicleStatePublisher;
class CarlaObstacleListPublisher;
class BasicSubscriber;
class BasicPublisher;

class ROS2 {
public:
  // deleting copy constructor for singleton
  ROS2(const ROS2 &obj) = delete;
  static std::shared_ptr<ROS2> GetInstance() {
    if (!_instance)
      _instance = std::shared_ptr<ROS2>(new ROS2);
    return _instance;
  }

  // general
  void Enable(bool enable);
  void Shutdown();
  bool IsEnabled() { return _enabled; }
  void SetFrame(uint64_t frame);
  void SetTimestamp(double timestamp);

  // actor registration API: replaces the legacy AddActorRosName /
  // GetActorRosName / GetActorParentRosName surface that PR-2 stubbed and
  // PR-4 retired. The plugin calls RegisterSensor / RegisterVehicle when an
  // actor spawns and Unregister* when it destroys; ROS2 builds the topic
  // names, owns per-sensor publishers + TF, and routes subscriber
  // callbacks. The vehicle gets exactly one control subscriber: the Ackermann
  // subscriber when enable_ackermann_control is true, otherwise the direct
  // VehicleControl one. The two control topics are mutually exclusive so they
  // cannot contend frame to frame.
  void RegisterSensor(
      void *actor, std::string ros_name, std::string frame_id, bool publish_tf);
  void UnregisterSensor(void *actor);
  void RegisterVehicle(
      void *actor, std::string ros_name, std::string frame_id, ActorCallback callback,
      bool enable_ackermann_control = false);
  void UnregisterVehicle(void *actor);

  // Robot dog (Nav2 migration) API: the actor subscribes to the global
  // /cmd_vel topic, moves kinematically on the UE side, and gets /odom
  // plus odom->base_link / base_link->laser_up TF published through
  // PublishRobotDogOdometry(). A lidar attached to a registered robot dog
  // is redirected to the fixed /scan/points topic with frame_id laser_up.
  void RegisterRobotDog(void *actor, ActorCallback callback);
  void PublishRobotDogOdometry(
      double x, double y, double z,
      double yaw_rad,
      double vx, double vy, double wz);

  // Topic-hierarchy seam used by the plugin's attach_actor path: tells ROS2
  // that `actor` should publish under `parent`'s ros_name prefix. Walking
  // the parent chain is the publisher-side concern.
  void AddActorParentRosName(void *actor, void *parent);

  // Demo subscriber callbacks (only compiled when WITH_ROS2_DEMO).
  void RemoveBasicSubscriberCallback(void *actor);
  void AddBasicSubscriberCallback(
      void *actor, std::string ros_name, ActorMessageCallback callback);

  // enabling streams to publish
  void EnableStream(carla::streaming::detail::stream_id_type id) {
    _publish_stream.insert(id);
  }
  bool IsStreamEnabled(carla::streaming::detail::stream_id_type id) {
    return _publish_stream.count(id) > 0;
  }
  void ResetStreams() { _publish_stream.clear(); }

  // receiving data to publish
  void ProcessDataFromCamera(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      int W, int H, float Fov,
      const carla::SharedBufferView buffer,
      void *actor = nullptr);
  void ProcessDataFromGNSS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::geom::GeoLocation &data,
      void *actor = nullptr);
  void ProcessDataFromIMU(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::geom::Vector3D accelerometer,
      carla::geom::Vector3D gyroscope,
      float compass,
      void *actor = nullptr);
  void ProcessDataFromDVS(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::SharedBufferView buffer,
      int W, int H, float Fov,
      void *actor = nullptr);
  void ProcessDataFromLidar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::LidarData &data,
      void *actor = nullptr);
  void ProcessDataFromSemanticLidar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      carla::sensor::data::SemanticLidarData &data,
      void *actor = nullptr);
  void ProcessDataFromRadar(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      const carla::sensor::data::RadarData &data,
      void *actor = nullptr);
  void ProcessDataFromObstacleDetection(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      AActor *first_actor,
      AActor *second_actor,
      float distance,
      void *actor = nullptr);
  void ProcessDataFromCollisionSensor(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      uint32_t other_actor,
      carla::geom::Vector3D impulse,
      void *actor);

  void ProcessDataFromVehicleData(
      uint64_t sensor_type,
      carla::streaming::detail::stream_id_type stream_id,
      const carla::geom::Transform sensor_transform,
      // InsData fields
      double longitude, double latitude, float altitude,
      float yaw, float pitch, float roll,
      float vx, float vy, float vt, float psd,
      float r, float p, float q,
      uint8_t gps_fix_state, uint8_t satellite_num, uint8_t gps_id,
      uint8_t nav_state, uint16_t nav_fault_code,
      bool is_valid, double timestamp,
      // VehicleState fields
      uint8_t work_state, uint8_t work_mode, uint8_t control_model,
      uint8_t battery_capacity, uint16_t voltage, uint16_t current,
      float speed, float angle, float brake, uint16_t fault_code,
      // ObstacleList fields
      const std::vector<ObstacleItemData>& obstacles,
      void *actor = nullptr);

private:
  struct ActorRegistration {
    std::string ros_name;
    std::string frame_id;
    bool publish_tf{true};
    // ros_name is used as-is under "rt/" instead of the
    // "rt/carla/[parent/]<ros_name>" hierarchy (robot dog topics).
    bool absolute_topic{false};
    bool is_robot_dog{false};
  };

  // Walks the actor's parent chain; if any ancestor is a registered robot
  // dog, rewrites the actor's registration so its data flows to the fixed
  // robot dog topics (lidar -> rt/scan/points, frame laser_up) and its own
  // TF stays disabled (the robot dog publishes the static TF instead).
  void RedirectToRobotDogTopicsIfNeeded(void *actor);

  // Resolves an actor's `rt/carla/[parent/]ros_name` base topic by walking the
  // parent chain. Returns empty if the actor is not registered.
  std::string BuildBaseTopicName(void *actor) const;
  std::string LookupRosName(void *actor) const;
  std::string LookupFrameId(void *actor) const;
  std::string BuildParentChain(void *actor) const;

  // Lazy-creates the per-sensor publisher matching `type` (an ESensors enum
  // declared in ROS2.cpp). Returns the BasePublisher pointer; the caller
  // dynamic_pointer_casts to the concrete subtype for typed Write() calls.
  std::shared_ptr<BasePublisher> GetOrCreateSensor(
      int type, carla::streaming::detail::stream_id_type id, void *actor);

  // Lazy-creates the per-sensor transform publisher, gated on the actor's
  // publish_tf flag (set at RegisterSensor time). Returns nullptr if the
  // sensor opted out.
  std::shared_ptr<CarlaTransformPublisher> GetOrCreateTransformPublisher(void *actor);

  // Camera-side counterpart of GetOrCreateSensor for publishers that inherit
  // CarlaCameraPublisher (the RGB / Depth / SS / IS / Normals / OpticalFlow
  // unified base). DVS uses its own composite via GetOrCreateSensor.
  template <typename CameraT>
  std::shared_ptr<CarlaCameraPublisher> GetOrCreateCameraSensor(
      carla::streaming::detail::stream_id_type id,
      void *actor,
      const std::string &default_prefix);

  // Resolves a `prefix__` placeholder by appending the stream id, persisting
  // the resolved name in `_registrations` so subsequent lookups see it.
  void ResolveAutoStreamSuffix(
      void *actor, const std::string &prefix, carla::streaming::detail::stream_id_type id);

  // singleton
  ROS2() = default;

  static std::shared_ptr<ROS2> _instance;

  bool _enabled{false};
  uint64_t _frame{0};
  int32_t _seconds{0};
  uint32_t _nanoseconds{0};

  std::unordered_map<void *, ActorRegistration> _registrations;
  std::unordered_map<void *, std::vector<void *>> _actor_parents;
  std::shared_ptr<CarlaClockPublisher> _clock_publisher;
  std::unordered_map<void *, std::shared_ptr<BasePublisher>> _publishers;
  std::unordered_map<void *, std::shared_ptr<CarlaCameraPublisher>> _camera_publishers;
  std::unordered_map<void *, std::shared_ptr<CarlaTransformPublisher>> _transforms;
  std::unordered_set<carla::streaming::detail::stream_id_type> _publish_stream;
  std::unordered_map<void *, ActorCallback> _actor_callbacks;
  std::unordered_multimap<void *, std::shared_ptr<BaseSubscriber>> _subscribers;

  // VehicleData sensor: 3 publishers per actor (INS, VehicleState, ObstacleList)
  struct VehicleDataPublishers {
    std::shared_ptr<CarlaInsDataPublisher> ins;
    std::shared_ptr<CarlaVehicleStatePublisher> vehicle_state;
    std::shared_ptr<CarlaObstacleListPublisher> obstacle_list;
  };
  std::unordered_map<void *, VehicleDataPublishers> _vehicle_data_publishers;
#if defined(WITH_ROS2_DEMO)
  std::shared_ptr<BasicSubscriber> _basic_subscriber;
  std::shared_ptr<BasicPublisher> _basic_publisher;
  std::unordered_map<void *, ActorMessageCallback> _actor_message_callbacks;
#endif

  // Robot dog: single instance of each (one dog per world today).
  void *_robot_dog_actor{nullptr};
  std::shared_ptr<CarlaOdometryPublisher> _robot_dog_odom_publisher;
  std::shared_ptr<CarlaTransformPublisher> _robot_dog_tf_publisher;
};

}  // namespace ros2
}  // namespace carla
