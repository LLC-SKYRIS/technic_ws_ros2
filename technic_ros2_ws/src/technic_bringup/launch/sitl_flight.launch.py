from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("technic_bringup"), "config", "flight.yaml"
    )
    start_agent = LaunchConfiguration("start_agent")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description="Start Micro XRCE-DDS Agent on UDP port 8888",
            ),
            ExecuteProcess(
                cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
                output="screen",
                condition=IfCondition(start_agent),
            ),
            Node(
                package="technic_flight",
                executable="flight_manager",
                name="flight_manager",
                output="screen",
                parameters=[config, {"use_sim_time": True}],
                emulate_tty=True,
            ),
        ]
    )
