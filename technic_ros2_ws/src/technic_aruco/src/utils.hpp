#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

namespace technic_aruco
{

inline void parseCameraInfo(
  const sensor_msgs::msg::CameraInfo & cinfo, cv::Mat & matrix, cv::Mat & dist)
{
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      matrix.at<double>(i, j) = cinfo.k[3 * i + j];
    }
  }
  dist = cv::Mat(cinfo.d, true);
}

// Rotation vector (axis * angle) as a quaternion.
inline tf2::Quaternion rvecToQuaternion(const cv::Vec3d & rvec)
{
  const double angle = cv::norm(rvec);
  tf2::Quaternion q(0, 0, 0, 1);
  if (angle > 1e-12) {
    const cv::Vec3d axis = rvec / angle;
    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
  }
  return q;
}

inline void quaternionToMsg(const tf2::Quaternion & q, geometry_msgs::msg::Quaternion & msg)
{
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}

inline tf2::Quaternion quaternionFromMsg(const geometry_msgs::msg::Quaternion & msg)
{
  return tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
}

inline void fillPose(geometry_msgs::msg::Pose & pose, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  pose.position.x = tvec[0];
  pose.position.y = tvec[1];
  pose.position.z = tvec[2];
  quaternionToMsg(rvecToQuaternion(rvec), pose.orientation);
}

inline void fillTransform(
  geometry_msgs::msg::Transform & transform, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  transform.translation.x = tvec[0];
  transform.translation.y = tvec[1];
  transform.translation.z = tvec[2];
  quaternionToMsg(rvecToQuaternion(rvec), transform.rotation);
}

inline void fillTranslation(geometry_msgs::msg::Vector3 & translation, const cv::Vec3d & tvec)
{
  translation.x = tvec[0];
  translation.y = tvec[1];
  translation.z = tvec[2];
}

inline tf2::Transform transformFromMsg(const geometry_msgs::msg::Transform & msg)
{
  return tf2::Transform(
    quaternionFromMsg(msg.rotation),
    tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

inline void transformToMsg(const tf2::Transform & transform, geometry_msgs::msg::Transform & msg)
{
  msg.translation.x = transform.getOrigin().x();
  msg.translation.y = transform.getOrigin().y();
  msg.translation.z = transform.getOrigin().z();
  quaternionToMsg(transform.getRotation(), msg.rotation);
}

inline tf2::Transform poseToTransform(const geometry_msgs::msg::Pose & pose)
{
  return tf2::Transform(
    quaternionFromMsg(pose.orientation),
    tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

inline void transformToPose(const tf2::Transform & transform, geometry_msgs::msg::Pose & pose)
{
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  quaternionToMsg(transform.getRotation(), pose.orientation);
}

inline bool isFlipped(const tf2::Quaternion & q)
{
  double yaw, pitch, roll;
  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
  return (std::abs(pitch) > M_PI / 2) || (std::abs(roll) > M_PI / 2);
}

/* Apply a vertical to an orientation */
inline void applyVertical(
  geometry_msgs::msg::Quaternion & orientation, const geometry_msgs::msg::Quaternion & vertical,
  bool flip_vertical = false, bool auto_flip = false)
{
  tf2::Quaternion _vertical = quaternionFromMsg(vertical);
  const tf2::Quaternion _orientation = quaternionFromMsg(orientation);

  if (flip_vertical || (auto_flip && !isFlipped(_orientation))) {
    tf2::Quaternion flip;
    flip.setRPY(M_PI, 0, 0);
    _vertical *= flip;  // flip vertical
  }

  const auto diff = tf2::Matrix3x3(_orientation).transposeTimes(tf2::Matrix3x3(_vertical));
  double roll, pitch, yaw;
  diff.getRPY(roll, pitch, yaw);
  tf2::Quaternion q;
  q.setRPY(0, 0, -yaw);
  _vertical = _vertical * q;  // set yaw from orientation to vertical
  quaternionToMsg(_vertical, orientation);  // set vertical to orientation
}

}  // namespace technic_aruco
