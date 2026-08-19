// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/ros2/publishers/CarlaOdometryPublisher.h"

#include "carla/Logging.h"
#include "carla/ros2/publishers/PublisherImpl.h"
#include "carla/ros2/types/Odometry.h"
#include "carla/ros2/types/OdometryPubSubTypes.h"

namespace carla {
namespace ros2 {

struct CarlaOdometryMsgTraits {
  using msg_type = nav_msgs::msg::Odometry;
  using msg_pubsub_type = nav_msgs::msg::OdometryPubSubType;
};

namespace {

// Small diagonal covariances: the pose comes from exact simulation
// integration (no wheel slip / drift), so Nav2 and AMCL only need finite,
// small values to weight odometry highly. Row-major 6x6 layout.
constexpr double kPoseVariance = 1e-4;
constexpr double kTwistVariance = 1e-4;

}  // namespace

CarlaOdometryPublisher::CarlaOdometryPublisher(
    std::string base_topic_name, std::string frame_id)
  : BasePublisher(std::move(base_topic_name), std::move(frame_id)),
    _impl(std::make_shared<PublisherImpl<CarlaOdometryMsgTraits>>()) {
  if (!_impl->Init(GetBaseTopicName())) {
    log_error("CarlaOdometryPublisher: failed to initialise writer for", GetBaseTopicName());
  }
}

CarlaOdometryPublisher::~CarlaOdometryPublisher() = default;

bool CarlaOdometryPublisher::Publish() {
  return _impl->Publish();
}

bool CarlaOdometryPublisher::Write(
    std::int32_t seconds,
    std::uint32_t nanoseconds,
    const std::string &child_frame_id,
    double x, double y, double z,
    double qw, double qx, double qy, double qz,
    double vx, double vy, double vz,
    double wx, double wy, double wz) {
  auto *message = _impl->GetMessage();
  message->header().stamp().sec(seconds);
  message->header().stamp().nanosec(nanoseconds);
  message->header().frame_id(GetFrameId());
  message->child_frame_id(child_frame_id);

  message->pose().pose().position().x(x);
  message->pose().pose().position().y(y);
  message->pose().pose().position().z(z);
  message->pose().pose().orientation().w(qw);
  message->pose().pose().orientation().x(qx);
  message->pose().pose().orientation().y(qy);
  message->pose().pose().orientation().z(qz);

  message->twist().twist().linear().x(vx);
  message->twist().twist().linear().y(vy);
  message->twist().twist().linear().z(vz);
  message->twist().twist().angular().x(wx);
  message->twist().twist().angular().y(wy);
  message->twist().twist().angular().z(wz);

  auto &pose_cov = message->pose().covariance();
  pose_cov.fill(0.0);
  pose_cov[0] = kPoseVariance;   // x
  pose_cov[7] = kPoseVariance;   // y
  pose_cov[14] = kPoseVariance;  // z
  pose_cov[21] = kPoseVariance;  // roll
  pose_cov[28] = kPoseVariance;  // pitch
  pose_cov[35] = kPoseVariance;  // yaw

  auto &twist_cov = message->twist().covariance();
  twist_cov.fill(0.0);
  twist_cov[0] = kTwistVariance;   // vx
  twist_cov[7] = kTwistVariance;   // vy
  twist_cov[14] = kTwistVariance;  // vz
  twist_cov[21] = kTwistVariance;  // wx
  twist_cov[28] = kTwistVariance;  // wy
  twist_cov[35] = kTwistVariance;  // wz

  return true;
}

}  // namespace ros2
}  // namespace carla
