#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "technic_flight/autopilot_backend.hpp"

namespace technic_flight
{

// Resolves user frames into the local ENU frame, following ROS 1 simple_offboard:
//
// - local frame ("map"): PX4 local position converted to ENU;
// - "base_link": vehicle pose with full attitude (published to TF, like MAVROS);
// - "body": vehicle position with yaw only (no roll/pitch);
// - "navigate_target": the current or last navigation target;
// - any other frame (e.g. "aruco_map"): looked up in TF.
//
// "base_link", "body" and "navigate_target" move with the vehicle, so goals given in them are
// frozen in the local frame at request time (their reference frame is local).
// Goals in other frames keep their own frame as reference and are re-transformed
// continuously, so they follow corrections of that frame.
class FrameResolver
{
public:
  explicit FrameResolver(rclcpp::Node & node);

  const std::string & local_frame() const {return local_frame_;}

  // Empty frame id means the local frame.
  std::string normalize(const std::string & frame_id) const;

  // Frame in which a goal given in `frame_id` is stored.
  std::string reference_frame(const std::string & frame_id) const;

  // Transform `pose` from frame `from` to frame `to`. A non-finite yaw is passed
  // through unchanged. Negative `timeout` selects the configured TF timeout.
  // Throws std::exception if the transform is not available.
  PositionSetpoint transform(
    const PositionSetpoint & pose,
    const std::string & from,
    const std::string & to,
    const VehicleSnapshot & state,
    double timeout = -1.0) const;

  // Rotate a free vector (e.g. a velocity) from frame `from` to frame `to`;
  // translations do not apply. Throws like transform().
  tf2::Vector3 rotate(
    const tf2::Vector3 & vector,
    const std::string & from,
    const std::string & to,
    const VehicleSnapshot & state,
    double timeout = -1.0) const;

  // Target in the local frame, published as the navigate_target frame.
  void set_navigate_target(const PositionSetpoint & target);

  // Broadcast the base_link, body and navigate_target frames.
  void publish_frames(const VehicleSnapshot & state);

private:
  tf2::Transform local_from(
    const std::string & frame, const VehicleSnapshot & state, double timeout) const;

  rclcpp::Node & node_;
  const std::string local_frame_;
  const std::string base_link_frame_;
  const std::string body_frame_;
  const std::string target_frame_;
  const double transform_timeout_;
  const bool publish_tf_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  mutable std::mutex mutex_;
  std::optional<PositionSetpoint> navigate_target_;
};

}  // namespace technic_flight
