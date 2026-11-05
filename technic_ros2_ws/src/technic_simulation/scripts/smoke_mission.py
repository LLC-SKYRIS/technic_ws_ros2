#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from technic_interfaces.action import Land, NavigateTo, Takeoff
from technic_interfaces.msg import FlightState, Telemetry


class SmokeMission(Node):
    def __init__(self):
        super().__init__("technic_simulation_smoke_mission")
        self.state = None
        self.telemetry = None
        self.create_subscription(
            FlightState, "/flight/state", self._state_callback, 10
        )
        telemetry_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Telemetry,
            "/flight/telemetry",
            self._telemetry_callback,
            telemetry_qos,
        )
        self.takeoff = ActionClient(self, Takeoff, "/flight/takeoff")
        self.navigate = ActionClient(self, NavigateTo, "/flight/navigate_to")
        self.land = ActionClient(self, Land, "/flight/land")

    def _state_callback(self, message):
        self.state = message

    def _telemetry_callback(self, message):
        self.telemetry = message

    def wait_until_ready(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (
                self.state is not None
                and self.state.connected
                and self.state.local_position_valid
            ):
                return True
        return False

    def execute(self, client, goal, label, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{label} action server is unavailable")

        self.get_logger().info(f"Starting {label}")
        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self.get_logger().info(
                f"{label} feedback: {feedback.feedback}"
            ),
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} goal was rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError(f"{label} action timed out")
        if not wrapped_result.result.success:
            raise RuntimeError(
                f"{label} failed: {wrapped_result.result.message}"
            )
        self.get_logger().info(f"{label}: {wrapped_result.result.message}")

    def run(self):
        if not self.wait_until_ready():
            raise RuntimeError(
                "PX4 did not provide a valid local position within 60 seconds"
            )

        takeoff_goal = Takeoff.Goal()
        takeoff_goal.height = 2.0
        takeoff_goal.speed = 0.5
        takeoff_goal.frame_id = "map"
        takeoff_goal.timeout = Duration(sec=30)
        self.execute(self.takeoff, takeoff_goal, "takeoff", 40.0)

        self.get_logger().info("Hovering for 10 seconds")
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.telemetry is None:
            raise RuntimeError("No flight telemetry received")

        origin_x = self.telemetry.pose.position.x
        origin_y = self.telemetry.pose.position.y
        altitude = self.telemetry.pose.position.z
        yaw = 0.0
        waypoints = [
            (origin_x + 1.0, origin_y, altitude),
            (origin_x + 1.0, origin_y + 1.0, altitude),
            (origin_x, origin_y + 1.0, altitude),
            (origin_x, origin_y, altitude),
        ]

        for index, (x, y, z) in enumerate(waypoints, start=1):
            goal = NavigateTo.Goal()
            goal.x = x
            goal.y = y
            goal.z = z
            goal.yaw = yaw if math.isfinite(yaw) else math.nan
            goal.speed = 0.5
            goal.frame_id = "map"
            goal.tolerance = 0.2
            goal.timeout = Duration(sec=30)
            self.execute(self.navigate, goal, f"waypoint {index}", 40.0)

        land_goal = Land.Goal()
        land_goal.timeout = Duration(sec=60)
        self.execute(self.land, land_goal, "land", 70.0)


def main():
    rclpy.init()
    mission = SmokeMission()
    exit_code = 0
    try:
        mission.run()
    except Exception as error:  # noqa: BLE001 - executable reports mission failures
        mission.get_logger().error(str(error))
        exit_code = 1
    finally:
        mission.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
