#include "technic_flight/frame_resolver.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>

namespace technic_flight
{
namespace
{

tf2::Transform pose_transform(double x, double y, double z, double yaw)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(rotation, tf2::Vector3(x, y, z));
}

geometry_msgs::msg::TransformStamped to_msg(
  const tf2::Transform & transform,
  const std::string & frame_id,
  const std::string & child_frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.transform.translation.x = transform.getOrigin().x();
  msg.transform.translation.y = transform.getOrigin().y();
  msg.transform.translation.z = transform.getOrigin().z();
  const auto rotation = transform.getRotation();
  msg.transform.rotation.x = rotation.x();
  msg.transform.rotation.y = rotation.y();
  msg.transform.rotation.z = rotation.z();
  msg.transform.rotation.w = rotation.w();
  return msg;
}

}  // namespace

FrameResolver::FrameResolver(rclcpp::Node & node)
: node_(node),
  local_frame_(node_.declare_parameter<std::string>("frames.local", "map")),
  base_link_frame_(node_.declare_parameter<std::string>("frames.base_link", "base_link")),
  body_frame_(node_.declare_parameter<std::string>("frames.body", "body")),
  target_frame_(node_.declare_parameter<std::string>("frames.navigate_target", "navigate_target")),
  transform_timeout_(node_.declare_parameter<double>("frames.transform_timeout", 0.5)),
  publish_tf_(node_.declare_parameter<bool>("frames.publish_tf", true))
{
  buffer_ = std::make_shared<tf2_ros::Buffer>(node_.get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  if (publish_tf_) {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }
}

std::string FrameResolver::normalize(const std::string & frame_id) const
{
  return frame_id.empty() ? local_frame_ : frame_id;
}

std::string FrameResolver::reference_frame(const std::string & frame_id) const
{
  const auto frame = normalize(frame_id);
  if (frame == base_link_frame_ || frame == body_frame_ || frame == target_frame_) {
    return local_frame_;
  }
  return frame;
}

PositionSetpoint FrameResolver::transform(
  const PositionSetpoint & pose,
  const std::string & from,
  const std::string & to,
  const VehicleSnapshot & state,
  double timeout) const
{
  const auto source = normalize(from);
  const auto target = normalize(to);
  if (source == target) {
    return pose;
  }

  const double wait = timeout < 0.0 ? transform_timeout_ : timeout;
  const tf2::Transform to_from =
    local_from(target, state, wait).inverse() * local_from(source, state, wait);

  const tf2::Vector3 position = to_from * tf2::Vector3(pose.x, pose.y, pose.z);
  double yaw = pose.yaw;
  if (std::isfinite(yaw)) {
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(to_from.getRotation() * rotation).getRPY(roll, pitch, yaw);
  }
  return PositionSetpoint{position.x(), position.y(), position.z(), yaw};
}

tf2::Vector3 FrameResolver::rotate(
  const tf2::Vector3 & vector,
  const std::string & from,
  const std::string & to,
  const VehicleSnapshot & state,
  double timeout) const
{
  const auto source = normalize(from);
  const auto target = normalize(to);
  if (source == target) {
    return vector;
  }
  const double wait = timeout < 0.0 ? transform_timeout_ : timeout;
  const tf2::Transform to_from =
    local_from(target, state, wait).inverse() * local_from(source, state, wait);
  return tf2::quatRotate(to_from.getRotation(), vector);
}

tf2::Transform FrameResolver::local_from(
  const std::string & frame, const VehicleSnapshot & state, double timeout) const
{
  if (frame == local_frame_) {
    return tf2::Transform::getIdentity();
  }
  if (frame == base_link_frame_) {
    if (!state.local_position_valid || !state.attitude_valid) {
      throw std::runtime_error("Vehicle pose is not valid, " + base_link_frame_ + " is unknown");
    }
    return tf2::Transform(
      tf2::Quaternion(state.qx, state.qy, state.qz, state.qw),
      tf2::Vector3(state.x, state.y, state.z));
  }
  if (frame == body_frame_) {
    if (!state.local_position_valid) {
      throw std::runtime_error("Local position is not valid, " + body_frame_ + " is unknown");
    }
    return pose_transform(state.x, state.y, state.z, state.yaw);
  }
  if (frame == target_frame_) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!navigate_target_) {
      throw std::runtime_error(target_frame_ + " is not set yet");
    }
    return pose_transform(
      navigate_target_->x, navigate_target_->y, navigate_target_->z, navigate_target_->yaw);
  }

  const auto msg = buffer_->lookupTransform(
    local_frame_, frame, tf2::TimePointZero, tf2::durationFromSec(timeout));
  const auto & t = msg.transform;
  return tf2::Transform(
    tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
    tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
}

void FrameResolver::set_navigate_target(const PositionSetpoint & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  navigate_target_ = target;
}

void FrameResolver::publish_frames(const VehicleSnapshot & state)
{
  if (!broadcaster_) {
    return;
  }

  const auto stamp = node_.now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  // base_link is published even before the position is valid (as MAVROS did):
  // ArUco needs the attitude to bootstrap the vision position estimate.
  if (state.attitude_valid && std::isfinite(state.x) && std::isfinite(state.y) &&
    std::isfinite(state.z))
  {
    transforms.push_back(
      to_msg(
        tf2::Transform(
          tf2::Quaternion(state.qx, state.qy, state.qz, state.qw),
          tf2::Vector3(state.x, state.y, state.z)),
        local_frame_, base_link_frame_, stamp));
  }
  if (state.local_position_valid) {
    transforms.push_back(
      to_msg(
        pose_transform(state.x, state.y, state.z, state.yaw),
        local_frame_, body_frame_, stamp));
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigate_target_) {
      transforms.push_back(
        to_msg(
          pose_transform(
            navigate_target_->x, navigate_target_->y, navigate_target_->z,
            navigate_target_->yaw),
          local_frame_, target_frame_, stamp));
    }
  }
  if (!transforms.empty()) {
    broadcaster_->sendTransform(transforms);
  }
}

}  // namespace technic_flight
