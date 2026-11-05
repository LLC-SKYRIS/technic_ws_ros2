#include "technic_flight/autopilot_backend.hpp"
#include "technic_flight/frame_resolver.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <technic_interfaces/action/land.hpp>
#include <technic_interfaces/action/navigate_to.hpp>
#include <technic_interfaces/action/takeoff.hpp>
#include <technic_interfaces/msg/flight_state.hpp>
#include <technic_interfaces/msg/telemetry.hpp>
#include <technic_interfaces/srv/arm.hpp>
#include <technic_interfaces/srv/release_control.hpp>
#include <technic_interfaces/srv/set_velocity.hpp>

namespace technic_flight
{
namespace
{

using namespace std::chrono_literals;

double distance(const VehicleSnapshot & state, const PositionSetpoint & target)
{
  return std::hypot(
    std::hypot(target.x - state.x, target.y - state.y),
    target.z - state.z);
}

double distance(const PositionSetpoint & from, const PositionSetpoint & to)
{
  return std::hypot(std::hypot(to.x - from.x, to.y - from.y), to.z - from.z);
}

// Point on the straight segment from `from` to `to`; yaw is taken from `to`.
PositionSetpoint interpolate(
  const PositionSetpoint & from, const PositionSetpoint & to, double fraction)
{
  return PositionSetpoint{
    from.x + (to.x - from.x) * fraction,
    from.y + (to.y - from.y) * fraction,
    from.z + (to.z - from.z) * fraction,
    to.yaw};
}

double seconds_since(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::string format_decimal(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", value);
  return buffer;
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration, double fallback)
{
  const auto seconds =
    static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  return seconds > 0.0 ? seconds : fallback;
}

class FlightManager final : public rclcpp::Node
{
public:
  using Takeoff = technic_interfaces::action::Takeoff;
  using NavigateTo = technic_interfaces::action::NavigateTo;
  using Land = technic_interfaces::action::Land;

  FlightManager()
  : Node("flight_manager"),
    heartbeat_hz_(declare_parameter<double>("offboard.heartbeat_hz", 10.0)),
    prestream_seconds_(declare_parameter<double>("offboard.prestream_seconds", 1.0)),
    takeoff_timeout_(declare_parameter<double>("flight.takeoff_timeout", 30.0)),
    navigate_timeout_(declare_parameter<double>("flight.navigate_timeout", 60.0)),
    land_timeout_(declare_parameter<double>("flight.land_timeout", 60.0)),
    offboard_timeout_(declare_parameter<double>("flight.offboard_timeout", 3.0)),
    arming_timeout_(declare_parameter<double>("flight.arming_timeout", 4.0)),
    position_tolerance_(declare_parameter<double>("flight.position_tolerance", 0.15)),
    default_takeoff_speed_(declare_parameter<double>("flight.takeoff_speed", 0.5)),
    allow_disarm_(declare_parameter<bool>("safety.allow_disarm_command", false)),
    max_velocity_(declare_parameter<double>("velocity.max_speed", 3.0)),
    frames_(*this)
  {
    const double tf_rate = declare_parameter<double>("frames.tf_rate", 50.0);
    if (tf_rate <= 0.0) {
      throw std::invalid_argument("frames.tf_rate must be positive");
    }

    if (heartbeat_hz_ < 2.0) {
      throw std::invalid_argument("offboard.heartbeat_hz must be at least 2 Hz");
    }

    const auto heartbeat_period = std::chrono::milliseconds(
      static_cast<int>(std::round(1000.0 / heartbeat_hz_)));
    const auto backend = declare_parameter<std::string>("autopilot.backend", "px4_dds");
    if (backend == "mavros") {
      backend_ = make_mavros_backend(*this, heartbeat_period);
    } else if (backend == "px4_dds") {
      backend_ = make_px4_backend(*this, heartbeat_period);
    } else {
      throw std::invalid_argument("autopilot.backend must be 'mavros' or 'px4_dds'");
    }
    RCLCPP_INFO(get_logger(), "Autopilot backend: %s", backend.c_str());

    state_pub_ = create_publisher<technic_interfaces::msg::FlightState>(
      "flight/state", rclcpp::QoS(1).reliable().transient_local());
    telemetry_pub_ = create_publisher<technic_interfaces::msg::Telemetry>(
      "flight/telemetry", rclcpp::SensorDataQoS());
    state_timer_ = create_wall_timer(100ms, [this]() {publish_state();});
    // Vehicle frames are needed at camera rate for ArUco (known vertical, VPE).
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tf_rate),
      [this]() {frames_.publish_frames(backend_->snapshot());});

    takeoff_server_ = rclcpp_action::create_server<Takeoff>(
      this, "flight/takeoff",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Takeoff::Goal> goal) {
        return validate_takeoff(*goal);
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> handle) {
        std::thread([this, handle]() {execute_takeoff(handle);}).detach();
      });

    navigate_server_ = rclcpp_action::create_server<NavigateTo>(
      this, "flight/navigate_to",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateTo::Goal> goal) {
        return validate_navigation(*goal);
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> handle) {
        std::thread([this, handle]() {execute_navigation(handle);}).detach();
      });

    land_server_ = rclcpp_action::create_server<Land>(
      this, "flight/land",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal>) {
        if (!backend_->snapshot().connected || !reserve_command()) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> handle) {
        std::thread([this, handle]() {execute_land(handle);}).detach();
      });

    arm_service_ = create_service<technic_interfaces::srv::Arm>(
      "flight/arm",
      [this](
        const std::shared_ptr<technic_interfaces::srv::Arm::Request> request,
        std::shared_ptr<technic_interfaces::srv::Arm::Response> response)
      {
        const auto state = backend_->snapshot();
        if (!state.connected || command_busy_.load()) {
          response->message =
            !state.connected ? "PX4 is not connected" : "A flight command is active";
          return;
        }
        if (!request->arm && !allow_disarm_) {
          response->message = "Disarm command is disabled by safety policy";
          return;
        }
        response->success = backend_->request_arm(request->arm);
        response->message = response->success ?
          (request->arm ? "Arm command sent" : "Disarm command sent") :
          "PX4 VehicleCommand service is not available";
      });

    release_service_ = create_service<technic_interfaces::srv::ReleaseControl>(
      "flight/release_control",
      [this](
        const std::shared_ptr<technic_interfaces::srv::ReleaseControl::Request> request,
        std::shared_ptr<technic_interfaces::srv::ReleaseControl::Response> response)
      {
        if (command_busy_.load()) {
          response->message = "Cancel the active action before releasing control";
          return;
        }
        clear_tracked_target();
        clear_velocity_command(true);  // never leave a velocity streaming
        if (request->fallback ==
          technic_interfaces::srv::ReleaseControl::Request::LAND)
        {
          response->success = backend_->request_land();
          response->message = response->success ?
            "PX4 land command sent" : "PX4 VehicleCommand service is not available";
          return;
        }
        if (request->fallback ==
          technic_interfaces::srv::ReleaseControl::Request::HOLD)
        {
          const auto state = backend_->snapshot();
          if (!state.connected || !state.armed || !state.offboard ||
            !state.local_position_valid)
          {
            response->message = "Position hold requires valid armed Offboard flight";
            return;
          }
          backend_->activate_position_control(
            PositionSetpoint{state.x, state.y, state.z, state.yaw});
          response->success = true;
          response->message = "Holding current position";
          return;
        }
        response->message = "Unknown release fallback";
      });

    velocity_service_ = create_service<technic_interfaces::srv::SetVelocity>(
      "flight/set_velocity",
      [this](
        const std::shared_ptr<technic_interfaces::srv::SetVelocity::Request> request,
        std::shared_ptr<technic_interfaces::srv::SetVelocity::Response> response)
      {
        response->message = set_velocity(*request);
        response->success = response->message.empty();
        if (response->success) {
          response->message = "Velocity setpoint accepted";
        }
      });
    // Expiry of velocity commands and following of TF frames such as aruco_map.
    velocity_timer_ = create_wall_timer(50ms, [this]() {update_velocity_command();});

    RCLCPP_INFO(get_logger(), "Technic flight manager is ready (safe/disarmed startup)");
  }

private:
  class CommandGuard
  {
  public:
    CommandGuard(
      std::atomic_bool & busy,
      std::string & active_command,
      std::mutex & mutex,
      std::string command)
    : busy_(busy), active_command_(active_command), mutex_(mutex)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_command_ = std::move(command);
    }

    ~CommandGuard()
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_command_.clear();
      }
      busy_.store(false);
    }

  private:
    std::atomic_bool & busy_;
    std::string & active_command_;
    std::mutex & mutex_;
  };

  rclcpp_action::GoalResponse validate_takeoff(const Takeoff::Goal & goal)
  {
    const auto state = backend_->snapshot();
    const bool valid_frame = goal.frame_id.empty() || goal.frame_id == "map";
    if (!state.connected || !state.local_position_valid || state.failsafe ||
      !std::isfinite(goal.height) || !std::isfinite(goal.speed) ||
      goal.height <= 0.0 || goal.speed < 0.0 || !valid_frame)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return reserve_command() ?
           rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse validate_navigation(const NavigateTo::Goal & goal)
  {
    // Arming and frame availability are checked on execution, where the
    // failure reason can be reported in the action result.
    const auto state = backend_->snapshot();
    const bool finite =
      std::isfinite(goal.x) && std::isfinite(goal.y) && std::isfinite(goal.z) &&
      std::isfinite(goal.speed) && goal.speed > 0.0;
    const bool valid_yaw = !(std::isinf(goal.yaw) && goal.yaw < 0.0);
    if (!state.connected || !state.local_position_valid || state.failsafe ||
      !finite || !valid_yaw)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return reserve_command() ?
           rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  bool reserve_command()
  {
    bool expected = false;
    if (!command_busy_.compare_exchange_strong(expected, true)) {
      return false;
    }
    clear_tracked_target();
    // An action takes over from a velocity command; until it streams its own
    // setpoints the vehicle holds its position instead of keeping the velocity.
    clear_velocity_command(true);
    return true;
  }

  // ------------------------------------------------------------ set_velocity

  // As ROS 1 simple_offboard set_velocity: the velocity is given in frame_id;
  // body, base_link and navigate_target are resolved at request time (like
  // navigate goals), TF frames such as aruco_map are followed continuously.
  struct VelocityCommand
  {
    std::string reference;   // frame of `velocity` and `yaw`
    tf2::Vector3 velocity;
    double yaw;              // NaN: hold `hold_yaw`
    double yaw_rate;         // NaN: no yaw rate control
    double hold_yaw;         // local frame
    std::chrono::steady_clock::time_point expires;
  };

  // Returns an error message, empty on success.
  std::string set_velocity(const technic_interfaces::srv::SetVelocity::Request & request)
  {
    const auto state = backend_->snapshot();
    if (!healthy_for_external_control(state)) {
      return "PX4 is not connected, local position is not valid or failsafe is active";
    }
    if (!state.armed || !state.offboard) {
      return "Vehicle is not armed in Offboard mode; take off with navigate_to first";
    }
    if (!std::isfinite(request.vx) || !std::isfinite(request.vy) || !std::isfinite(request.vz)) {
      return "vx, vy and vz must be finite";
    }
    if (std::isinf(request.yaw) || std::isinf(request.yaw_rate)) {
      return "yaw and yaw_rate must be finite or NaN";
    }
    if (std::isfinite(request.yaw) && std::isfinite(request.yaw_rate)) {
      return "Set either yaw or yaw_rate, the other one must be NaN";
    }
    const double valid_for = duration_seconds(request.valid_for, 0.0);
    if (valid_for <= 0.0) {
      return "valid_for must be positive";
    }
    const double speed = std::hypot(std::hypot(request.vx, request.vy), request.vz);
    if (max_velocity_ > 0.0 && speed > max_velocity_) {
      return "Speed " + format_decimal(speed) + " m/s exceeds velocity.max_speed " +
             format_decimal(max_velocity_) + " m/s";
    }

    const std::string frame_id = frames_.normalize(request.frame_id);
    VelocityCommand command;
    command.reference = frames_.reference_frame(frame_id);
    command.yaw_rate = request.yaw_rate;
    command.expires = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(valid_for));
    VelocitySetpoint setpoint;
    try {
      command.velocity = frames_.rotate(
        tf2::Vector3(request.vx, request.vy, request.vz), frame_id, command.reference, state);
      command.yaw = std::isfinite(request.yaw) ?
        frames_.transform(
        PositionSetpoint{0.0, 0.0, 0.0, request.yaw}, frame_id, command.reference, state).yaw :
        std::numeric_limits<double>::quiet_NaN();
    } catch (const std::exception & error) {
      return "Can't transform from " + frame_id + " to " + frames_.local_frame() + ": " +
             error.what();
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_busy_.load()) {
      return "A flight command is active (" + active_command_ + "); cancel it first";
    }
    // NaN yaw keeps the heading being commanded now (as navigate_to does).
    if (velocity_command_ && !std::isfinite(velocity_command_->yaw_rate)) {
      command.hold_yaw = velocity_command_->hold_yaw;
      if (std::isfinite(velocity_command_->yaw)) {
        try {
          command.hold_yaw = local_velocity(*velocity_command_, state).yaw;
        } catch (const std::exception &) {
        }
      }
    } else if (const auto streamed = backend_->active_setpoint()) {
      command.hold_yaw = streamed->yaw;
    } else {
      command.hold_yaw = state.yaw;
    }
    try {
      setpoint = local_velocity(command, state);
    } catch (const std::exception & error) {
      return std::string("Can't transform the velocity: ") + error.what();
    }
    tracked_target_.reset();
    velocity_command_ = command;
    backend_->activate_velocity_control(setpoint);
    return "";
  }

  VelocitySetpoint local_velocity(const VelocityCommand & command, const VehicleSnapshot & state)
  {
    const auto & local = frames_.local_frame();
    const auto v = frames_.rotate(command.velocity, command.reference, local, state, 0.05);
    VelocitySetpoint setpoint{v.x(), v.y(), v.z()};
    if (std::isfinite(command.yaw_rate)) {
      setpoint.yaw_rate = command.yaw_rate;
    } else if (std::isfinite(command.yaw)) {
      setpoint.yaw = frames_.transform(
        PositionSetpoint{0.0, 0.0, 0.0, command.yaw}, command.reference, local, state, 0.05).yaw;
    } else {
      setpoint.yaw = command.hold_yaw;
    }
    return setpoint;
  }

  // Drop the velocity command; with `hold` the vehicle keeps its current position.
  void clear_velocity_command(bool hold)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!velocity_command_) {
      return;
    }
    velocity_command_.reset();
    if (hold) {
      hold_position(backend_->snapshot());
    }
  }

  // Position hold at the current position and heading; command_mutex_ is held.
  void hold_position(const VehicleSnapshot & state)
  {
    const PositionSetpoint hold{state.x, state.y, state.z, state.yaw};
    backend_->activate_position_control(hold);
    frames_.set_navigate_target(hold);
  }

  void update_velocity_command()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!velocity_command_) {
      return;
    }
    const auto state = backend_->snapshot();
    if (!healthy_for_external_control(state) || !state.armed || !state.offboard) {
      // Control belongs to the pilot or PX4 failsafe now.
      velocity_command_.reset();
      backend_->deactivate_offboard_stream();
      RCLCPP_WARN(get_logger(), "Velocity control stopped: Offboard flight is no longer possible");
      return;
    }
    if (std::chrono::steady_clock::now() > velocity_command_->expires) {
      velocity_command_.reset();
      hold_position(state);
      RCLCPP_INFO(get_logger(), "Velocity command expired; holding position");
      return;
    }
    if (velocity_command_->reference != frames_.local_frame()) {
      try {
        backend_->activate_velocity_control(local_velocity(*velocity_command_, state));
      } catch (const std::exception & error) {
        // Keep the last velocity until the frame is back or the command expires.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "Can't transform: %s", error.what());
      }
    }
  }

  void clear_tracked_target()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    tracked_target_.reset();
  }

  // After arrival, keep following a target given in a TF frame such as
  // aruco_map, as ROS 1 does by re-transforming the setpoint every cycle.
  void refresh_tracked_target(const VehicleSnapshot & state)
  {
    if (command_busy_.load()) {
      return;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    const auto current = backend_->active_setpoint();
    if (!tracked_target_ || !current) {
      return;
    }
    try {
      auto target = frames_.transform(
        tracked_target_->pose, tracked_target_->frame, frames_.local_frame(), state, 0.0);
      if (!std::isfinite(target.yaw)) {
        target.yaw = current->yaw;
      }
      backend_->update_position_setpoint(target);
      frames_.set_navigate_target(target);
    } catch (const std::exception &) {
      // Transform temporarily unavailable: keep the last setpoint.
    }
  }

  enum class WaitResult
  {
    kDone,
    kTimeout,
    kCanceled,
    kUnhealthy,
    kShutdown,
  };

  struct EngageResult
  {
    WaitResult result;
    std::string message;
  };

  // Poll the vehicle state at 10 Hz until `done` holds, the goal is canceled,
  // external control becomes unsafe or the timeout expires.
  template<typename GoalHandleT, typename Predicate>
  WaitResult wait_for_vehicle(
    const std::shared_ptr<GoalHandleT> & handle, double timeout, Predicate done)
  {
    const auto started = std::chrono::steady_clock::now();
    rclcpp::WallRate loop(10.0);
    while (rclcpp::ok()) {
      const auto state = backend_->snapshot();
      if (done(state)) {
        return WaitResult::kDone;
      }
      if (handle->is_canceling()) {
        return WaitResult::kCanceled;
      }
      if (!healthy_for_external_control(state)) {
        return WaitResult::kUnhealthy;
      }
      if (seconds_since(started) > timeout) {
        return WaitResult::kTimeout;
      }
      loop.sleep();
    }
    return WaitResult::kShutdown;
  }

  // ROS 1 auto_arm: stream `hold` as the setpoint, switch PX4 to Offboard and
  // arm, waiting for PX4 to confirm each step. On failure the stream is stopped.
  template<typename GoalHandleT>
  EngageResult engage_offboard(
    const std::shared_ptr<GoalHandleT> & handle, const PositionSetpoint & hold)
  {
    backend_->activate_position_control(hold);
    const auto fail = [this](WaitResult result, const std::string & message) {
        backend_->deactivate_offboard_stream();
        return EngageResult{
          result,
          result == WaitResult::kUnhealthy ? "Flight safety condition failed" : message};
      };

    if (!backend_->snapshot().offboard) {
      std::this_thread::sleep_for(std::chrono::duration<double>(prestream_seconds_));
      if (!backend_->request_offboard()) {
        return fail(WaitResult::kTimeout, "PX4 VehicleCommand service is unavailable");
      }
      const auto result = wait_for_vehicle(
        handle, offboard_timeout_, [](const VehicleSnapshot & s) {return s.offboard;});
      if (result != WaitResult::kDone) {
        return fail(result, "PX4 did not switch to Offboard mode");
      }
    }

    if (!backend_->snapshot().armed) {
      if (!backend_->request_arm(true)) {
        return fail(WaitResult::kTimeout, "PX4 arm command could not be sent");
      }
      const auto result = wait_for_vehicle(
        handle, arming_timeout_, [](const VehicleSnapshot & s) {return s.armed;});
      if (result != WaitResult::kDone) {
        return fail(result, "PX4 did not arm; check preflight checks in QGroundControl");
      }
    }
    return EngageResult{WaitResult::kDone, ""};
  }

  void execute_takeoff(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> handle)
  {
    CommandGuard guard(command_busy_, active_command_, command_mutex_, "takeoff");
    const auto goal = handle->get_goal();
    const auto started = std::chrono::steady_clock::now();
    const auto initial = backend_->snapshot();
    const PositionSetpoint start{initial.x, initial.y, initial.z, initial.yaw};
    const PositionSetpoint target{start.x, start.y, start.z + goal->height, start.yaw};
    const double speed = goal->speed > 0.0 ? goal->speed : default_takeoff_speed_;
    const double timeout = duration_seconds(goal->timeout, takeoff_timeout_);
    const double climb_time = goal->height / speed;

    if (climb_time >= timeout) {
      finish_takeoff(
        handle, false,
        "Climb at " + format_decimal(speed) + " m/s needs " + format_decimal(climb_time) +
        " s, which exceeds the takeoff timeout", false);
      return;
    }

    frames_.set_navigate_target(target);

    // Hold the start position until PX4 confirms Offboard and arming.
    const auto engaged = engage_offboard(handle, start);
    if (engaged.result == WaitResult::kCanceled) {
      finish_takeoff(handle, false, "Takeoff cancelled before climb", true);
      return;
    }
    if (engaged.result != WaitResult::kDone) {
      if (engaged.result != WaitResult::kShutdown) {
        finish_takeoff(handle, false, engaged.message, false);
      }
      return;
    }

    // Like ROS 1 navigate: the setpoint moves from the start point at the
    // requested speed, independently of where the vehicle currently is.
    const auto climb_started = std::chrono::steady_clock::now();
    rclcpp::WallRate loop(10.0);
    while (rclcpp::ok()) {
      const auto state = backend_->snapshot();

      if (handle->is_canceling()) {
        request_land_and_release_offboard();
        finish_takeoff(handle, false, "Takeoff cancelled; PX4 land requested", true);
        return;
      }
      if (!healthy_for_external_control(state)) {
        request_land_and_release_offboard();
        finish_takeoff(handle, false, "Flight safety condition failed", false);
        return;
      }
      if (!state.armed) {
        backend_->deactivate_offboard_stream();
        finish_takeoff(handle, false, "Vehicle disarmed during takeoff", false);
        return;
      }
      if (!state.offboard) {
        // Mode changed by the pilot or PX4: control belongs to them now.
        backend_->deactivate_offboard_stream();
        finish_takeoff(handle, false, "Offboard mode was left during takeoff", false);
        return;
      }
      if (seconds_since(started) > timeout) {
        request_land_and_release_offboard();
        finish_takeoff(handle, false, "Takeoff timed out; PX4 land requested", false);
        return;
      }

      const double passed = std::min(seconds_since(climb_started) / climb_time, 1.0);
      backend_->update_position_setpoint(interpolate(start, target, passed));

      auto feedback = std::make_shared<Takeoff::Feedback>();
      feedback->current_height = state.z - start.z;
      feedback->remaining_distance = std::max(0.0, target.z - state.z);
      feedback->flight_state = technic_interfaces::msg::FlightState::TAKING_OFF;
      handle->publish_feedback(feedback);

      if (passed >= 1.0 && feedback->remaining_distance <= position_tolerance_) {
        finish_takeoff(handle, true, "Takeoff complete", false);
        return;
      }
      loop.sleep();
    }
  }

  void execute_navigation(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> handle)
  {
    CommandGuard guard(command_busy_, active_command_, command_mutex_, "navigate_to");
    const auto goal = handle->get_goal();
    const auto initial = backend_->snapshot();
    const std::string frame_id = frames_.normalize(goal->frame_id);
    const std::string reference = frames_.reference_frame(frame_id);
    const std::string & local = frames_.local_frame();
    const bool towards = std::isinf(goal->yaw);  // +Inf; -Inf is rejected on validation

    if (!goal->auto_arm && (!initial.armed || !initial.offboard)) {
      abort_navigation(handle, "Vehicle is not armed in Offboard mode, use auto_arm");
      return;
    }

    // Freeze the goal in its reference frame: body and navigate_target are
    // resolved now, TF frames such as aruco_map are tracked while flying.
    PositionSetpoint goal_reference;
    PositionSetpoint target;
    try {
      goal_reference = frames_.transform(
        PositionSetpoint{
          goal->x, goal->y, goal->z,
          std::isfinite(goal->yaw) ? goal->yaw : std::numeric_limits<double>::quiet_NaN()},
        frame_id, reference, initial);
      target = frames_.transform(goal_reference, reference, local, initial);
    } catch (const std::exception & error) {
      abort_navigation(
        handle, "Can't transform from " + frame_id + " to " + local + ": " + error.what());
      return;
    }

    // auto_arm from a non-Offboard state starts from the vehicle position (as
    // ROS 1 does); otherwise continue from the streamed setpoint.
    const bool engage = goal->auto_arm && (!initial.armed || !initial.offboard);
    const auto streamed = backend_->active_setpoint();
    const PositionSetpoint start = (!engage && streamed) ?
      *streamed : PositionSetpoint{initial.x, initial.y, initial.z, initial.yaw};

    const auto with_yaw = [&](PositionSetpoint point) {
        if (towards) {
          const double dx = point.x - start.x;
          const double dy = point.y - start.y;
          point.yaw = std::hypot(dx, dy) > 0.01 ? std::atan2(dy, dx) : start.yaw;
        } else if (!std::isfinite(point.yaw)) {
          point.yaw = start.yaw;
        }
        return point;
      };
    target = with_yaw(target);

    const double tolerance =
      goal->tolerance > 0.0 ? goal->tolerance : position_tolerance_;
    const double timeout = duration_seconds(goal->timeout, navigate_timeout_);
    const double travel_time = distance(start, target) / goal->speed;

    if (travel_time >= timeout) {
      abort_navigation(
        handle,
        "Flight at " + format_decimal(goal->speed) + " m/s needs " +
        format_decimal(travel_time) + " s, which exceeds the navigation timeout");
      return;
    }

    frames_.set_navigate_target(target);

    if (engage) {
      const auto engaged = engage_offboard(handle, start);
      if (engaged.result == WaitResult::kCanceled) {
        auto result = std::make_shared<NavigateTo::Result>();
        result->message = "Navigation cancelled before arming";
        handle->canceled(result);
        return;
      }
      if (engaged.result != WaitResult::kDone) {
        if (engaged.result != WaitResult::kShutdown) {
          abort_navigation(handle, engaged.message);
        }
        return;
      }
    } else if (!streamed) {
      // Offboard is held by someone else's stream; take it over from here.
      backend_->activate_position_control(start);
    }

    const auto started = std::chrono::steady_clock::now();
    rclcpp::WallRate loop(10.0);

    while (rclcpp::ok()) {
      const auto state = backend_->snapshot();
      const double elapsed = seconds_since(started);

      if (handle->is_canceling()) {
        backend_->update_position_setpoint(
          PositionSetpoint{state.x, state.y, state.z, state.yaw});
        auto result = std::make_shared<NavigateTo::Result>();
        result->message = "Navigation cancelled; holding current position";
        handle->canceled(result);
        return;
      }
      if (!healthy_for_external_control(state) || !state.armed || !state.offboard) {
        abort_navigation(handle, "Flight safety condition failed");
        return;
      }
      if (elapsed > timeout) {
        backend_->update_position_setpoint(
          PositionSetpoint{state.x, state.y, state.z, state.yaw});
        abort_navigation(handle, "Navigation timed out; holding current position");
        return;
      }

      // Follow a moving reference frame (e.g. aruco_map corrections); keep the
      // last known target while its transform is unavailable.
      if (reference != local) {
        try {
          target = with_yaw(frames_.transform(goal_reference, reference, local, state, 0.05));
          frames_.set_navigate_target(target);
        } catch (const std::exception & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000, "Can't transform: %s", error.what());
        }
      }

      const double passed = travel_time > 0.0 ? std::min(elapsed / travel_time, 1.0) : 1.0;
      backend_->update_position_setpoint(interpolate(start, target, passed));

      const double remaining = distance(state, target);
      if (passed >= 1.0 && remaining <= tolerance) {
        if (reference != local) {
          std::lock_guard<std::mutex> lock(command_mutex_);
          tracked_target_ = TrackedTarget{reference, goal_reference};
        }
        auto result = std::make_shared<NavigateTo::Result>();
        result->success = true;
        result->message = "Navigation complete";
        handle->succeed(result);
        return;
      }

      auto feedback = std::make_shared<NavigateTo::Feedback>();
      feedback->remaining_distance = remaining;
      feedback->elapsed_seconds = elapsed;
      feedback->flight_state = technic_interfaces::msg::FlightState::NAVIGATING;
      handle->publish_feedback(feedback);
      loop.sleep();
    }
  }

  void execute_land(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> handle)
  {
    CommandGuard guard(command_busy_, active_command_, command_mutex_, "land");
    const double timeout = duration_seconds(handle->get_goal()->timeout, land_timeout_);
    const auto started = std::chrono::steady_clock::now();
    bool stream_stopped = false;
    if (!backend_->request_land()) {
      auto result = std::make_shared<Land::Result>();
      result->message = "PX4 VehicleCommand service is unavailable";
      handle->abort(result);
      return;
    }
    rclcpp::WallRate loop(10.0);

    while (rclcpp::ok()) {
      const auto state = backend_->snapshot();
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

      if (!state.offboard && !stream_stopped) {
        backend_->deactivate_offboard_stream();
        stream_stopped = true;
      }

      auto feedback = std::make_shared<Land::Feedback>();
      feedback->current_height = state.z;
      feedback->flight_state = technic_interfaces::msg::FlightState::LANDING;
      handle->publish_feedback(feedback);

      if (!state.armed) {
        backend_->deactivate_offboard_stream();
        auto result = std::make_shared<Land::Result>();
        result->success = true;
        result->message = "Landing complete; vehicle disarmed";
        handle->succeed(result);
        return;
      }
      if (!state.connected || elapsed > timeout) {
        auto result = std::make_shared<Land::Result>();
        result->message = !state.connected ? "PX4 connection lost" : "Landing timed out";
        handle->abort(result);
        return;
      }
      loop.sleep();
    }
  }

  bool healthy_for_external_control(const VehicleSnapshot & state) const
  {
    return state.connected && state.local_position_valid && !state.failsafe;
  }

  void request_land_and_release_offboard()
  {
    if (!backend_->request_land()) {
      backend_->deactivate_offboard_stream();
      return;
    }

    for (int attempt = 0; attempt < 20 && rclcpp::ok(); ++attempt) {
      if (!backend_->snapshot().offboard) {
        break;
      }
      std::this_thread::sleep_for(100ms);
    }
    backend_->deactivate_offboard_stream();
  }

  void finish_takeoff(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Takeoff>> & handle,
    bool success,
    const std::string & message,
    bool canceled)
  {
    auto result = std::make_shared<Takeoff::Result>();
    result->success = success;
    result->message = message;
    if (canceled) {
      handle->canceled(result);
    } else if (success) {
      handle->succeed(result);
    } else {
      handle->abort(result);
    }
  }

  void abort_navigation(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateTo>> & handle,
    const std::string & message)
  {
    auto result = std::make_shared<NavigateTo::Result>();
    result->message = message;
    handle->abort(result);
  }

  void publish_state()
  {
    const auto snapshot = backend_->snapshot();
    refresh_tracked_target(snapshot);
    technic_interfaces::msg::FlightState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.connected = snapshot.connected;
    msg.armed = snapshot.armed;
    msg.offboard = snapshot.offboard;
    msg.local_position_valid = snapshot.local_position_valid;
    msg.failsafe = snapshot.failsafe;
    msg.autopilot_mode = snapshot.mode;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      msg.active_command = active_command_;
      if (msg.active_command.empty() && velocity_command_) {
        msg.active_command = "set_velocity";
      }
    }

    if (!snapshot.connected) {
      msg.state = technic_interfaces::msg::FlightState::DISCONNECTED;
      msg.state_label = "DISCONNECTED";
    } else if (snapshot.failsafe) {
      msg.state = technic_interfaces::msg::FlightState::FAILSAFE;
      msg.state_label = "FAILSAFE";
    } else if (!snapshot.armed) {
      msg.state = technic_interfaces::msg::FlightState::DISARMED;
      msg.state_label = "DISARMED";
    } else if (msg.active_command == "takeoff") {
      msg.state = technic_interfaces::msg::FlightState::TAKING_OFF;
      msg.state_label = "TAKING_OFF";
    } else if (msg.active_command == "navigate_to" || msg.active_command == "set_velocity") {
      msg.state = technic_interfaces::msg::FlightState::NAVIGATING;
      msg.state_label = "NAVIGATING";
    } else if (msg.active_command == "land") {
      msg.state = technic_interfaces::msg::FlightState::LANDING;
      msg.state_label = "LANDING";
    } else if (snapshot.offboard) {
      msg.state = technic_interfaces::msg::FlightState::HOVERING;
      msg.state_label = "HOVERING";
    } else {
      msg.state = technic_interfaces::msg::FlightState::ARMED;
      msg.state_label = "ARMED";
    }
    state_pub_->publish(msg);

    technic_interfaces::msg::Telemetry telemetry;
    telemetry.header = msg.header;
    telemetry.connected = snapshot.connected;
    telemetry.armed = snapshot.armed;
    telemetry.offboard = snapshot.offboard;
    telemetry.local_position_valid = snapshot.local_position_valid;
    telemetry.autopilot_mode = snapshot.mode;
    telemetry.pose.position.x = snapshot.x;
    telemetry.pose.position.y = snapshot.y;
    telemetry.pose.position.z = snapshot.z;
    telemetry.pose.orientation.z = std::sin(snapshot.yaw / 2.0);
    telemetry.pose.orientation.w = std::cos(snapshot.yaw / 2.0);
    telemetry.twist.linear.x = snapshot.vx;
    telemetry.twist.linear.y = snapshot.vy;
    telemetry.twist.linear.z = snapshot.vz;
    telemetry.battery_voltage = std::numeric_limits<float>::quiet_NaN();
    telemetry.battery_remaining = std::numeric_limits<float>::quiet_NaN();
    telemetry.localization_age_seconds = std::numeric_limits<double>::quiet_NaN();
    telemetry_pub_->publish(telemetry);
  }

  const double heartbeat_hz_;
  const double prestream_seconds_;
  const double takeoff_timeout_;
  const double navigate_timeout_;
  const double land_timeout_;
  const double offboard_timeout_;
  const double arming_timeout_;
  const double position_tolerance_;
  const double default_takeoff_speed_;
  const bool allow_disarm_;
  const double max_velocity_;

  struct TrackedTarget
  {
    std::string frame;
    PositionSetpoint pose;  // yaw NaN keeps the current yaw setpoint
  };

  FrameResolver frames_;

  std::shared_ptr<AutopilotBackend> backend_;
  std::atomic_bool command_busy_{false};
  std::mutex command_mutex_;
  std::string active_command_;
  std::optional<TrackedTarget> tracked_target_;  // guarded by command_mutex_
  std::optional<VelocityCommand> velocity_command_;  // guarded by command_mutex_

  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<NavigateTo>::SharedPtr navigate_server_;
  rclcpp_action::Server<Land>::SharedPtr land_server_;
  rclcpp::Service<technic_interfaces::srv::Arm>::SharedPtr arm_service_;
  rclcpp::Service<technic_interfaces::srv::ReleaseControl>::SharedPtr release_service_;
  rclcpp::Service<technic_interfaces::srv::SetVelocity>::SharedPtr velocity_service_;
  rclcpp::Publisher<technic_interfaces::msg::FlightState>::SharedPtr state_pub_;
  rclcpp::Publisher<technic_interfaces::msg::Telemetry>::SharedPtr telemetry_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr velocity_timer_;
};

}  // namespace
}  // namespace technic_flight

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  // Keep a strong reference for the whole executor lifetime. Executor::add_node()
  // does not take ownership of a temporary node shared pointer.
  auto node = std::make_shared<technic_flight::FlightManager>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
