"""Technic web UI (port of the web part of technic/launch/technic.launch).

  http://<drone>:8000/technic/   pages (web_server, ROS 1 roswww)
  http://<drone>:8000/technic/docs/  documentation (technic_docs package)
  http://<drone>:8000/technic_blocks/  blocks programming (technic_blocks node)
  ws://<drone>:9090              rosbridge + rosapi for roslib.js
  http://<drone>:8080            web_video_server (camera streams, snapshots)
  http://<drone>:57575           Butterfly web terminal (ROS 1 butterfly.service)
  web_api                        ROS 1 style /navigate, /land, /get_telemetry,
                                 /exec, /genmap ... over the ROS 2 flight API
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def _docs_mount():
    """/technic/docs/ from the technic_docs package, when it is installed."""
    try:
        share = get_package_share_directory("technic_docs")
    except PackageNotFoundError:
        return []
    return ["technic/docs=" + os.path.join(share, "docs")]


def _web_server(context):
    mounts = [m for m in LaunchConfiguration("mounts").perform(context).split(",") if m.strip()]
    if IfCondition(LaunchConfiguration("docs")).evaluate(context):
        mounts += _docs_mount()
    if IfCondition(LaunchConfiguration("blocks")).evaluate(context):
        mounts.append("technic_blocks=" + os.path.join(
            get_package_share_directory("technic_web"), "technic_blocks", "www"))
    return [
        Node(
            package="technic_web",
            executable="web_server",
            name="web_server",
            output="screen",
            respawn=True,
            parameters=[{
                "port": int(LaunchConfiguration("web_port").perform(context)),
                "mounts": mounts or [""],
            }],
        )
    ]


def _butterfly(context):
    # Not a ROS node: a wrapper that execs butterfly.server.py (pip/pipx package).
    script = os.path.join(FindPackagePrefix("technic_web").perform(context),
                          "lib", "technic_web", "butterfly_terminal")
    return [
        ExecuteProcess(
            cmd=[script,
                 "--host", LaunchConfiguration("butterfly_host").perform(context),
                 "--port", LaunchConfiguration("butterfly_port").perform(context),
                 "--login", LaunchConfiguration("butterfly_login").perform(context)],
            name="butterfly",
            output="screen",
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration("butterfly")),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("web_port", default_value="8000"),
            DeclareLaunchArgument(
                "mounts",
                default_value="",
                description='extra static directories, comma separated "url_path=directory", '
                "e.g. games=/home/orangepi/games",
            ),
            DeclareLaunchArgument(
                "docs",
                default_value="true",
                description="documentation at /technic/docs/ (technic_docs package)",
            ),
            DeclareLaunchArgument(
                "blocks",
                default_value="true",
                description="blocks programming page /technic_blocks/ and its technic_blocks node",
            ),
            DeclareLaunchArgument(
                "blocks_programs_dir",
                default_value="~/technic_blocks_programs",
                description="where the blocks page saves user programs",
            ),
            DeclareLaunchArgument("rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("web_video_server", default_value="true"),
            DeclareLaunchArgument("web_video_port", default_value="8080"),
            DeclareLaunchArgument(
                "butterfly",
                default_value="true",
                description="Butterfly web terminal (Open Web Terminal tile); "
                "needs pipx install butterfly",
            ),
            DeclareLaunchArgument(
                "butterfly_port",
                default_value="57575",
                description="the index page links to this port (or ?butterfly_port=)",
            ),
            DeclareLaunchArgument("butterfly_host", default_value="0.0.0.0"),
            DeclareLaunchArgument(
                "butterfly_login",
                default_value="true",
                description="true: ask for the user password (as ROS 1); "
                "false: a shell for everyone on the network without a password",
            ),
            OpaqueFunction(function=_web_server),
            OpaqueFunction(function=_butterfly),
            Node(
                package="technic_web",
                executable="web_api",
                name="web_api",
                output="screen",
                respawn=True,
            ),
            Node(
                package="technic_web",
                executable="technic_blocks",
                name="technic_blocks",
                output="screen",
                respawn=True,
                parameters=[{"programs_dir": LaunchConfiguration("blocks_programs_dir")}],
                condition=IfCondition(LaunchConfiguration("blocks")),
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                respawn=True,
                parameters=[{"port": LaunchConfiguration("rosbridge_port"), "address": ""}],
                condition=IfCondition(LaunchConfiguration("rosbridge")),
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="screen",
                respawn=True,
                condition=IfCondition(LaunchConfiguration("rosbridge")),
            ),
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                respawn=True,
                respawn_delay=5.0,
                parameters=[{"port": LaunchConfiguration("web_video_port"), "address": "0.0.0.0"}],
                condition=IfCondition(LaunchConfiguration("web_video_server")),
            ),
        ]
    )
