#include "technic_flight/autopilot_backend.hpp"

#include <cmath>
#include <cstdint>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace technic_flight
{
namespace
{

// MAV_STATE values from which PX4 reports a failsafe-like condition.
constexpr std::uint8_t kMavStateCritical = 5;
constexpr std::uint8_t kMavStateFlightTermination = 8;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// mavros does the ENU <-> NED and FLU <-> FRD conversions, so this backend works
// in ROS ENU / base_link FLU only (as ROS 1 technic did).
class MavrosBackend final : public AutopilotBackend
{
public:
  MavrosBackend(
    rclcpp::Node & node,
    std::chrono::milliseconds heartbeat_period)
  : node_(node),
    status_timeout_(node_.declare_parameter<double>("mavros.status_timeout", 3.0)),
    position_timeout_(node_.declare_parameter<double>("mavros.position_timeout", 0.5))
  {
    const auto ns = node_.declare_parameter<std::string>("mavros.namespace", "/mavros/mavros");
    const auto vision_pose_topic =
      node_.declare_parameter<std::string>("px4.vision_pose_topic", "vision_pose/pose");
    const auto sensor_qos = rclcpp::SensorDataQoS();

    setpoint_pub_ = node_.create_publisher<mavros_msgs::msg::PositionTarget>(
      ns + "/setpoint_raw/local", 10);
    vision_pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
      ns + "/vision_pose/pose", 10);
    set_mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>(ns + "/set_mode");
    arming_client_ = node_.create_client<mavros_msgs::srv::CommandBool>(ns + "/cmd/arming");

    state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
      ns + "/state", rclcpp::QoS(10),
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (msg->connected) {
          RCLCPP_INFO_ONCE(node_.get_logger(), "mavros connected to PX4");
        }
        fcu_connected_ = msg->connected;
        snapshot_.armed = msg->armed;
        snapshot_.mode = msg->mode;
        snapshot_.offboard = msg->mode == "OFFBOARD";
        snapshot_.failsafe =
          msg->system_status >= kMavStateCritical &&
          msg->system_status <= kMavStateFlightTermination;
        last_status_ = std::chrono::steady_clock::now();
        status_received_ = true;
      });

    // PX4 sends LOCAL_POSITION_NED only while its local position is valid.
    pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/local_position/pose", sensor_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.x = msg->pose.position.x;
        snapshot_.y = msg->pose.position.y;
        snapshot_.z = msg->pose.position.z;
        snapshot_.yaw = yaw_from_quaternion(msg->pose.orientation);
        last_position_ = std::chrono::steady_clock::now();
        position_received_ = true;
      });

    velocity_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
      ns + "/local_position/velocity_local", sensor_qos,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.vx = msg->twist.linear.x;
        snapshot_.vy = msg->twist.linear.y;
        snapshot_.vz = msg->twist.linear.z;
      });

    // base_link (FLU) orientation in the local ENU frame.
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      ns + "/imu/data", sensor_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        const auto & q = msg->orientation;
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.attitude_valid = std::isfinite(q.w) && msg->orientation_covariance[0] != -1.0;
        snapshot_.qx = q.x;
        snapshot_.qy = q.y;
        snapshot_.qz = q.z;
        snapshot_.qw = q.w;
      });

    // Vision pose estimate (e.g. ArUco VPE): base_link pose in the local ENU frame.
    vision_pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      vision_pose_topic, 1,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        vision_pose_pub_->publish(*msg);
      });

    heartbeat_timer_ = node_.create_wall_timer(
      heartbeat_period, [this]() {publish_offboard_stream();});
  }

  VehicleSnapshot snapshot() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    auto result = snapshot_;
    result.connected =
      status_received_ && fcu_connected_ &&
      (now - last_status_ <= std::chrono::duration<double>(status_timeout_));
    result.local_position_valid =
      position_received_ &&
      (now - last_position_ <= std::chrono::duration<double>(position_timeout_));
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
    return set_mode("OFFBOARD");
  }

  bool request_arm(bool arm) override
  {
    if (!arming_client_->service_is_ready()) {
      RCLCPP_ERROR(node_.get_logger(), "mavros arming service is not available");
      return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arming_client_->async_send_request(
      request,
      [logger = node_.get_logger(), arm](
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
      {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            logger, "PX4 rejected %s with result %u",
            arm ? "arming" : "disarming", response->result);
        }
      });
    return true;
  }

  bool request_land() override
  {
    // As ROS 1 technic: land at the current position.
    return set_mode("AUTO.LAND");
  }

private:
  bool set_mode(const std::string & mode)
  {
    if (!set_mode_client_->service_is_ready()) {
      RCLCPP_ERROR(node_.get_logger(), "mavros set_mode service is not available");
      return false;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    set_mode_client_->async_send_request(
      request,
      [logger = node_.get_logger(), mode](
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
      {
        if (!future.get()->mode_sent) {
          RCLCPP_WARN(logger, "mavros could not send mode %s", mode.c_str());
        }
      });
    return true;
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

    using mavros_msgs::msg::PositionTarget;
    PositionTarget target{};
    target.header.stamp = node_.get_clock()->now();
    // ENU values; mavros converts them to PX4 LOCAL_NED.
    target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    if (velocity_mode) {
      const bool rate = std::isfinite(velocity.yaw_rate);
      target.type_mask =
        PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ |
        PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ |
        (rate ? PositionTarget::IGNORE_YAW : PositionTarget::IGNORE_YAW_RATE);
      target.velocity.x = velocity.vx;
      target.velocity.y = velocity.vy;
      target.velocity.z = velocity.vz;
      target.yaw = rate ? 0.0F : static_cast<float>(velocity.yaw);
      target.yaw_rate = rate ? static_cast<float>(velocity.yaw_rate) : 0.0F;
      setpoint_pub_->publish(target);
      return;
    }
    target.type_mask =
      PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ |
      PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ |
      PositionTarget::IGNORE_YAW_RATE;
    target.position.x = setpoint.x;
    target.position.y = setpoint.y;
    target.position.z = setpoint.z;
    target.yaw = static_cast<float>(setpoint.yaw);
    setpoint_pub_->publish(target);
  }

  rclcpp::Node & node_;
  const double status_timeout_;
  const double position_timeout_;

  mutable std::mutex mutex_;
  VehicleSnapshot snapshot_;
  PositionSetpoint position_setpoint_;
  VelocitySetpoint velocity_setpoint_;
  bool velocity_mode_{false};
  bool stream_active_{false};
  bool fcu_connected_{false};
  bool status_received_{false};
  bool position_received_{false};
  std::chrono::steady_clock::time_point last_status_{};
  std::chrono::steady_clock::time_point last_position_{};

  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace

std::shared_ptr<AutopilotBackend> make_mavros_backend(
  rclcpp::Node & node,
  std::chrono::milliseconds heartbeat_period)
{
  return std::make_shared<MavrosBackend>(node, heartbeat_period);
}

}  // namespace technic_flight
