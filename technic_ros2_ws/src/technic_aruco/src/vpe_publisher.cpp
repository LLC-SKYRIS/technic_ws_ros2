// ROS 2 port of technic/vpe_publisher: turn the detected markers map pose into
// a vision pose estimate for the autopilot and publish the map -> aruco_map offset.

#include <functional>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <technic_interfaces/msg/telemetry.hpp>

#include "utils.hpp"

namespace technic_aruco
{

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;
using technic_interfaces::msg::Telemetry;

class VpePublisher : public rclcpp::Node
{
public:
  explicit VpePublisher(const rclcpp::NodeOptions & options)
  : Node("vpe_publisher", options)
  {
    frame_id_ = declare_parameter("frame_id", std::string(""));  // name for used visual pose frame
    offset_frame_id_ = declare_parameter("offset_frame_id", std::string(""));  // published offset frame
    local_frame_id_ = declare_parameter("local_frame_id", std::string("map"));
    child_frame_id_ = declare_parameter("child_frame_id", std::string("base_link"));
    offset_timeout_ = rclcpp::Duration::from_seconds(declare_parameter("offset_timeout", 3.0));
    transform_timeout_ = declare_parameter("transform_timeout", 0.02);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_br_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), frame_id_.empty() ? "using data topic" : "using data from TF");

    pose_sub_ = create_subscription<PoseStamped>(
      "~/pose", 1, [this](PoseStamped::ConstSharedPtr msg) {callback(msg->header, msg->pose);});
    pose_cov_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "~/pose_cov", 1,
      [this](PoseWithCovarianceStamped::ConstSharedPtr msg) {callback(msg->header, msg->pose.pose);});
    vpe_pub_ = create_publisher<PoseStamped>("~/vpe", 1);
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        reset_flag_ = true;
        res->success = true;
      });

    // publish_zero is the old name of force_init
    const bool force_init = declare_parameter("force_init", false);
    const bool publish_zero = declare_parameter("publish_zero", false);
    if (force_init || publish_zero) {
      // publish zero to initialize the local position
      publish_zero_timeout_ = rclcpp::Duration::from_seconds(
        declare_parameter("force_init_timeout", 5.0));
      publish_zero_duration_ = rclcpp::Duration::from_seconds(
        declare_parameter("force_init_duration", 5.0));
      // ROS 1 used mavros/local_position/pose, which PX4 only sends with a valid
      // local position; the flight manager telemetry carries that flag explicitly.
      telemetry_sub_ = create_subscription<Telemetry>(
        "telemetry", rclcpp::SensorDataQoS(),
        [this](Telemetry::ConstSharedPtr msg) {
          if (msg->local_position_valid) {
            last_local_position_ = now();
          }
        });
      zero_timer_ = create_timer(
        std::chrono::milliseconds(100), std::bind(&VpePublisher::publishZero, this));
    }

    RCLCPP_INFO(get_logger(), "ready");
  }

private:
  bool reset_flag_ = true;  // offset should be reset on the start
  std::string local_frame_id_, frame_id_, child_frame_id_, offset_frame_id_;
  rclcpp::Duration offset_timeout_{0, 0};
  double transform_timeout_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr vpe_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  PoseStamped vpe_;
  TransformStamped offset_;
  rclcpp::Subscription<Telemetry>::SharedPtr telemetry_sub_;
  rclcpp::TimerBase::SharedPtr zero_timer_;
  rclcpp::Duration publish_zero_timeout_{0, 0};
  rclcpp::Duration publish_zero_duration_{0, 0};
  std::optional<rclcpp::Time> last_local_position_;
  std::optional<rclcpp::Time> got_local_pos_;

  void publishZero()
  {
    const auto current = now();
    const rclcpp::Time vpe_stamp(vpe_.header.stamp, get_clock()->get_clock_type());
    if (vpe_stamp.nanoseconds() != 0 && current - vpe_stamp < publish_zero_timeout_) {
      return;  // have vpe
    }

    if (last_local_position_ && current - *last_local_position_ < publish_zero_timeout_) {
      // have local position
      if (!got_local_pos_) {
        RCLCPP_INFO(get_logger(), "got local position");
        got_local_pos_ = current;
      }

      if (current - *got_local_pos_ > publish_zero_duration_) {
        return;  // stop publishing zero
      }
    } else {
      // lost local position
      got_local_pos_.reset();
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "publish zero");
    PoseStamped zero;
    zero.header.frame_id = local_frame_id_;
    zero.header.stamp = current;
    zero.pose.orientation.w = 1;
    vpe_pub_->publish(zero);
  }

  TransformStamped lookup(
    const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp)
  {
    return tf_buffer_->lookupTransform(
      target, source, tf2_ros::fromMsg(stamp), tf2::durationFromSec(transform_timeout_));
  }

  void callback(const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
  {
    try {
      if (!frame_id_.empty()) {
        // get VPE transform from TF
        const auto transform = lookup(frame_id_, child_frame_id_, header.stamp);
        transformToPose(transformFromMsg(transform.transform), vpe_.pose);
      } else {
        vpe_.pose = pose;
      }

      // offset
      if (!offset_frame_id_.empty()) {
        const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
        const rclcpp::Time last(vpe_.header.stamp, get_clock()->get_clock_type());
        if (reset_flag_ || stamp - last > offset_timeout_) {
          // calculate the offset
          if (!frame_id_.empty()) {
            // calculate from TF
            offset_ = lookup(local_frame_id_, frame_id_, header.stamp);
            offset_.child_frame_id = offset_frame_id_;
          } else {
            // calculate transform between pose in vpe frame and pose in local frame
            auto local_pose = lookup(local_frame_id_, child_frame_id_, header.stamp);
            // keep yaw only
            tf2::Quaternion yaw;
            yaw.setRPY(0, 0, tf2::getYaw(quaternionFromMsg(local_pose.transform.rotation)));
            quaternionToMsg(yaw, local_pose.transform.rotation);

            const tf2::Transform offset =
              poseToTransform(vpe_.pose).inverseTimes(transformFromMsg(local_pose.transform));
            transformToMsg(offset, offset_.transform);
            offset_.header.frame_id = local_frame_id_;
            offset_.header.stamp = header.stamp;
            offset_.child_frame_id = offset_frame_id_;
          }

          static_br_->sendTransform(offset_);
          reset_flag_ = false;
          RCLCPP_INFO(get_logger(), "offset reset");
        }
        // apply the offset
        transformToPose(
          transformFromMsg(offset_.transform) * poseToTransform(vpe_.pose), vpe_.pose);
      }

      vpe_.header.frame_id = local_frame_id_;
      vpe_.header.stamp = header.stamp;
      vpe_pub_->publish(vpe_);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
    }
  }
};

}  // namespace technic_aruco

RCLCPP_COMPONENTS_REGISTER_NODE(technic_aruco::VpePublisher)
