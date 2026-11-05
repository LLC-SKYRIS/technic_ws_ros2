#pragma once

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace technic_flight
{

struct VehicleSnapshot
{
  bool connected{false};
  bool armed{false};
  bool offboard{false};
  bool local_position_valid{false};
  bool failsafe{false};
  double x{0.0};  // ENU east, metres
  double y{0.0};  // ENU north, metres
  double z{0.0};  // ENU up, metres
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw{0.0};  // ENU, radians
  // Full attitude of base_link (FLU) in the local ENU frame.
  bool attitude_valid{false};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  std::string mode;
};

struct PositionSetpoint
{
  double x{0.0};  // ENU
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

// Velocity setpoint in the local ENU frame. Exactly one of yaw and yaw_rate is
// finite: yaw holds a heading, yaw_rate turns at a rate (ENU, counterclockwise).
struct VelocitySetpoint
{
  double vx{0.0};  // ENU, metres per second
  double vy{0.0};
  double vz{0.0};
  double yaw{std::numeric_limits<double>::quiet_NaN()};       // ENU, radians
  double yaw_rate{std::numeric_limits<double>::quiet_NaN()};  // radians per second
};

class AutopilotBackend
{
public:
  virtual ~AutopilotBackend() = default;

  virtual VehicleSnapshot snapshot() const = 0;
  // Position setpoint currently streamed to the autopilot, if the stream is active
  // in position mode (none while velocity setpoints are streamed).
  virtual std::optional<PositionSetpoint> active_setpoint() const = 0;
  virtual void activate_position_control(const PositionSetpoint & setpoint) = 0;
  virtual void update_position_setpoint(const PositionSetpoint & setpoint) = 0;
  // Start or update streaming velocity setpoints (Offboard velocity control).
  virtual void activate_velocity_control(const VelocitySetpoint & setpoint) = 0;
  virtual void deactivate_offboard_stream() = 0;
  virtual bool request_offboard() = 0;
  virtual bool request_arm(bool arm) = 0;
  virtual bool request_land() = 0;
};

// PX4 over uXRCE-DDS (px4_msgs, /fmu/* topics).
std::shared_ptr<AutopilotBackend> make_px4_backend(
  rclcpp::Node & node,
  std::chrono::milliseconds heartbeat_period);

// PX4 over MAVLink through mavros (/mavros/* topics and services).
std::shared_ptr<AutopilotBackend> make_mavros_backend(
  rclcpp::Node & node,
  std::chrono::milliseconds heartbeat_period);

}  // namespace technic_flight
