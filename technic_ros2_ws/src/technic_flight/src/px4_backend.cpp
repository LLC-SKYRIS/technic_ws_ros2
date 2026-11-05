#include "technic_flight/autopilot_backend.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace technic_flight
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;

double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double yaw_enu_to_ned(double yaw_enu)
{
  return normalize_angle(kHalfPi - yaw_enu);
}

// Both conversions are 180 degree rotations, i.e. their own inverses:
// ENU <-> NED world frames and FLU <-> FRD body frames.
const tf2::Quaternion kEnuNed(M_SQRT1_2, M_SQRT1_2, 0.0, 0.0);
const tf2::Quaternion kFluFrd(1.0, 0.0, 0.0, 0.0);

// Orientation of an FRD body in NED <-> orientation of an FLU body in ENU.
tf2::Quaternion convert_orientation(const tf2::Quaternion & q)
{
  return kEnuNed * q * kFluFrd;
}

class Px4Backend final : public AutopilotBackend
{
public:
  Px4Backend(
    rclcpp::Node & node,
    std::chrono::milliseconds heartbeat_period)
  : node_(node),
    status_timeout_(node_.declare_parameter<double>("px4.status_timeout", 2.0)),
    target_system_(node_.declare_parameter<int>("px4.target_system", 1)),
    target_component_(node_.declare_parameter<int>("px4.target_component", 1))
  {
    const auto prefix = node_.declare_parameter<std::string>("px4.topic_prefix", "");
    const auto vision_pose_topic =
      node_.declare_parameter<std::string>("px4.vision_pose_topic", "vision_pose/pose");
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    // Request VOLATILE durability so the subscriber is compatible with both
    // VOLATILE and TRANSIENT_LOCAL PX4 publishers. PX4 telemetry is best-effort.
    const auto output_qos = rclcpp::SensorDataQoS();

    offboard_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
      prefix + "/fmu/in/offboard_control_mode", input_qos);
    trajectory_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      prefix + "/fmu/in/trajectory_setpoint", input_qos);
    visual_odometry_pub_ = node_.create_publisher<px4_msgs::msg::VehicleOdometry>(
      prefix + "/fmu/in/vehicle_visual_odometry", input_qos);
    command_client_ = node_.create_client<px4_msgs::srv::VehicleCommand>(
      prefix + "/fmu/vehicle_command");

    status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
      prefix + "/fmu/out/vehicle_status_v1", output_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO_ONCE(node_.get_logger(), "PX4 vehicle status stream connected");
        snapshot_.armed =
          msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        snapshot_.offboard =
          msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        snapshot_.failsafe = msg->failsafe;
        snapshot_.mode = mode_label(msg->nav_state);
        last_status_ = std::chrono::steady_clock::now();
        status_received_ = true;
      });

    local_position_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      prefix + "/fmu/out/vehicle_local_position_v1", output_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.local_position_valid = msg->xy_valid && msg->z_valid;
        // PX4 local position is NED; the flight core is ROS ENU.
        snapshot_.x = msg->y;
        snapshot_.y = msg->x;
        snapshot_.z = -msg->z;
        snapshot_.vx = msg->vy;
        snapshot_.vy = msg->vx;
        snapshot_.vz = -msg->vz;
        snapshot_.yaw = yaw_enu_to_ned(msg->heading);
      });

    attitude_sub_ = node_.create_subscription<px4_msgs::msg::VehicleAttitude>(
      prefix + "/fmu/out/vehicle_attitude", output_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        // PX4 q is the FRD body orientation in NED, (w, x, y, z).
        const auto q = convert_orientation(tf2::Quaternion(msg->q[1], msg->q[2], msg->q[3], msg->q[0]));
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.attitude_valid = std::isfinite(q.w());
        snapshot_.qx = q.x();
        snapshot_.qy = q.y();
        snapshot_.qz = q.z();
        snapshot_.qw = q.w();
      });

    // Vision pose estimate (e.g. ArUco VPE): base_link pose in the local ENU frame.
    vision_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      vision_pose_topic, 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {publish_vision_pose(*msg);});

    heartbeat_timer_ = node_.create_wall_timer(
      heartbeat_period, [this]() {publish_offboard_stream();});
  }

  VehicleSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = snapshot_;
    result.connected =
      status_received_ &&
      (std::chrono::steady_clock::now() - last_status_ <=
      std::chrono::duration<double>(status_timeout_));
    return result;
  }

  std::optional<PositionSetpoint> active_setpoint() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_active_ || velocity_mode_) {
      return std::nullopt;
    }
    return position_setpoint_;
  }

  void activate_position_control(const PositionSetpoint & setpoint) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position_setpoint_ = setpoint;
    velocity_mode_ = false;
    stream_active_ = true;
  }

  void update_position_setpoint(const PositionSetpoint & setpoint) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position_setpoint_ = setpoint;
    velocity_mode_ = false;
  }

  void activate_velocity_control(const VelocitySetpoint & setpoint) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_setpoint_ = setpoint;
    velocity_mode_ = true;
    stream_active_ = true;
  }

  void deactivate_offboard_stream() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stream_active_ = false;
  }

  bool request_offboard() override
  {
    return publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
  }

  bool request_arm(bool arm) override
  {
    return publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      arm ? 1.0F : 0.0F);
  }

  bool request_land() override
  {
    return publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

private:
  static std::string mode_label(std::uint8_t nav_state)
  {
    if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      return "OFFBOARD";
    }
    if (nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
      return "AUTO.LAND";
    }
    return std::to_string(nav_state);
  }

  std::uint64_t timestamp_us() const
  {
    return static_cast<std::uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
  }

  void publish_vision_pose(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & p = pose.pose.position;
    const auto & o = pose.pose.orientation;
    const auto q = convert_orientation(tf2::Quaternion(o.x, o.y, o.z, o.w));
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::VehicleOdometry odometry{};
    odometry.timestamp = timestamp_us();
    odometry.timestamp_sample =
      static_cast<std::uint64_t>(rclcpp::Time(pose.header.stamp).nanoseconds() / 1000);
    odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odometry.position = {
      static_cast<float>(p.y), static_cast<float>(p.x), static_cast<float>(-p.z)};
    odometry.q = {
      static_cast<float>(q.w()), static_cast<float>(q.x()),
      static_cast<float>(q.y()), static_cast<float>(q.z())};
    odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    odometry.velocity = {nan, nan, nan};
    odometry.angular_velocity = {nan, nan, nan};
    // Unknown variances: EKF2 falls back to EKF2_EVP_NOISE / EKF2_EVA_NOISE.
    odometry.position_variance = {nan, nan, nan};
    odometry.orientation_variance = {nan, nan, nan};
    odometry.velocity_variance = {nan, nan, nan};
    visual_odometry_pub_->publish(odometry);
  }

  void publish_offboard_stream()
  {
    PositionSetpoint setpoint;
    VelocitySetpoint velocity;
    bool velocity_mode = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stream_active_) {
        return;
      }
      setpoint = position_setpoint_;
      velocity = velocity_setpoint_;
      velocity_mode = velocity_mode_;
    }
    const float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp = timestamp_us();
    mode.position = !velocity_mode;
    mode.velocity = velocity_mode;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    offboard_mode_pub_->publish(mode);

    px4_msgs::msg::TrajectorySetpoint trajectory{};
    trajectory.timestamp = timestamp_us();
    if (velocity_mode) {
      // PX4 velocity control: position NaN, velocity NED.
      trajectory.position = {nan, nan, nan};
      trajectory.velocity = {
        static_cast<float>(velocity.vy),
        static_cast<float>(velocity.vx),
        static_cast<float>(-velocity.vz)};
      trajectory.yaw = std::isfinite(velocity.yaw) ?
        static_cast<float>(yaw_enu_to_ned(velocity.yaw)) : nan;
      // ENU yaw rate is counterclockwise, NED yawspeed clockwise.
      trajectory.yawspeed = std::isfinite(velocity.yaw_rate) ?
        static_cast<float>(-velocity.yaw_rate) : nan;
    } else {
      trajectory.position = {
        static_cast<float>(setpoint.y),
        static_cast<float>(setpoint.x),
        static_cast<float>(-setpoint.z)};
      trajectory.velocity = {nan, nan, nan};
      trajectory.yaw = static_cast<float>(yaw_enu_to_ned(setpoint.yaw));
      trajectory.yawspeed = nan;
    }
    trajectory.acceleration = {nan, nan, nan};
    trajectory_pub_->publish(trajectory);
  }

  bool publish_vehicle_command(
    std::uint32_t command,
    float param1 = 0.0F,
    float param2 = 0.0F)
  {
    if (!command_client_->service_is_ready()) {
      RCLCPP_ERROR(
        node_.get_logger(), "PX4 VehicleCommand service is not available");
      return false;
    }

    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = static_cast<std::uint8_t>(target_system_);
    msg.target_component = static_cast<std::uint8_t>(target_component_);
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    request->request = msg;
    command_client_->async_send_request(
      request,
      [logger = node_.get_logger()](
        rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
      {
        const auto & ack = future.get()->reply;
        if (ack.result !=
          px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED &&
          ack.result !=
          px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS)
        {
          RCLCPP_WARN(
            logger, "PX4 rejected vehicle command %u with result %u",
            ack.command, ack.result);
        }
      });
    return true;
  }

  rclcpp::Node & node_;
  const double status_timeout_;
  const int target_system_;
  const int target_component_;

  mutable std::mutex mutex_;
  VehicleSnapshot snapshot_;
  PositionSetpoint position_setpoint_;
  VelocitySetpoint velocity_setpoint_;
  bool velocity_mode_{false};
  bool stream_active_{false};
  bool status_received_{false};
  std::chrono::steady_clock::time_point last_status_{};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odometry_pub_;
  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr command_client_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace

std::shared_ptr<AutopilotBackend> make_px4_backend(
  rclcpp::Node & node,
  std::chrono::milliseconds heartbeat_period)
{
  return std::make_shared<Px4Backend>(node, heartbeat_period);
}

}  // namespace technic_flight
