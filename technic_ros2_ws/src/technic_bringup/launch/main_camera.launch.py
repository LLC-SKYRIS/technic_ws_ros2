"""Main camera: base_link -> main_camera_optical and the V4L2 driver.

Port of technic/launch/main_camera.launch. The driver publishes
/main_camera/image_raw and /main_camera/camera_info in frame main_camera_optical.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# rmw_cyclonedds with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST gives each process a
# participant index and allows only 32 of them; one launch is ~25 processes, so a
# second launch (or leftovers of a killed one) makes nodes fail with "Failed to find
# a free participant index". The same value must be set in every shell (README).
CYCLONEDDS_URI_DEFAULT = (
    '<CycloneDDS><Domain id="any"><Discovery>'
    '<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
    '</Discovery></Domain></CycloneDDS>'
)

# x y z yaw pitch roll of base_link -> main_camera_optical, as in ROS 1.
CAMERA_POSES = {
    ("down", "backward"): (0.05, 0.0, -0.07, -1.5707963, 0.0, 3.1415926),
    ("down", "forward"): (0.05, 0.0, -0.07, 1.5707963, 0.0, 3.1415926),
    # Up-facing camera, 13 cm forward and 10 cm above base_link.
    ("up", "backward"): (0.13, 0.0, 0.10, 1.5707963, 0.0, 0.0),
    ("up", "forward"): (0.13, 0.0, 0.10, -1.5707963, 0.0, 0.0),
    ("forward", None): (0.03, 0.0, 0.05, -1.5707963, 0.0, -1.5707963),
    ("backward", None): (-0.03, 0.0, 0.05, 1.5707963, 0.0, -1.5707963),
}

# Camera position of the Gazebo model (technic_description model.sdf).
SIMULATOR_TRANSLATION = (0.055, 0.0, -0.03)


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _nodes(context):
    direction_z = LaunchConfiguration("direction_z").perform(context)
    direction_y = LaunchConfiguration("direction_y").perform(context)
    simulator = _as_bool(context, "simulator")

    key = (direction_z, None if direction_z in ("forward", "backward") else direction_y)
    if key not in CAMERA_POSES:
        raise RuntimeError(
            "unsupported camera direction: direction_z=%s direction_y=%s" % (direction_z, direction_y)
        )
    x, y, z, yaw, pitch, roll = CAMERA_POSES[key]
    if simulator:
        x, y, z = SIMULATOR_TRANSLATION

    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="main_camera_frame",
            arguments=[
                "--x", str(x), "--y", str(y), "--z", str(z),
                "--yaw", str(yaw), "--pitch", str(pitch), "--roll", str(roll),
                "--frame-id", "base_link", "--child-frame-id", "main_camera_optical",
            ],
            parameters=[{"use_sim_time": simulator}],
        )
    ]

    if not simulator:
        width = int(LaunchConfiguration("width").perform(context))
        height = int(LaunchConfiguration("height").perform(context))
        nodes.append(
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                namespace="main_camera",
                name="main_camera",
                output="screen",
                respawn=True,
                parameters=[
                    {
                        "video_device": LaunchConfiguration("device").perform(context),
                        "image_size": [width, height],
                        "camera_frame_id": "main_camera_optical",
                        "camera_info_url": LaunchConfiguration("camera_info_url").perform(context),
                    }
                ],
            )
        )
    return nodes


def generate_launch_description():
    camera_info = os.path.join(
        get_package_share_directory("technic_bringup"),
        "config", "camera_info", "fisheye_cam_320x240.yaml",
    )
    return LaunchDescription(
        [
            # Cyclone DDS: large images (camera) without the multi-second stalls of
            # Fast DDS IP fragmentation. All ROS processes must use the same RMW.
            DeclareLaunchArgument("rmw", default_value="rmw_cyclonedds_cpp"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", LaunchConfiguration("rmw")),
            # All ROS nodes run on the drone (QGC uses mavros TCP), so keep DDS on
            # loopback: Cyclone DDS otherwise picks one network interface, which may
            # be down or switching (Wi-Fi AP/client) at boot.
            DeclareLaunchArgument(
                "discovery_range",
                default_value="LOCALHOST",
                description="ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST, SUBNET (ROS tools "
                "on a laptop over Wi-Fi), OFF or SYSTEM_DEFAULT",
            ),
            SetEnvironmentVariable(
                "ROS_AUTOMATIC_DISCOVERY_RANGE", LaunchConfiguration("discovery_range")
            ),
            SetEnvironmentVariable(
                "CYCLONEDDS_URI", os.environ.get("CYCLONEDDS_URI", CYCLONEDDS_URI_DEFAULT)
            ),
            DeclareLaunchArgument(
                "direction_z", default_value="down",
                description="direction the camera points: down, up, forward, backward",
            ),
            DeclareLaunchArgument(
                "direction_y", default_value="backward",
                description="direction the camera cable points: backward, forward",
            ),
            DeclareLaunchArgument("device", default_value="/dev/video0"),
            DeclareLaunchArgument("width", default_value="320"),
            DeclareLaunchArgument("height", default_value="240"),
            DeclareLaunchArgument(
                "camera_info_url", default_value="file://" + camera_info,
                description="calibration matching width x height",
            ),
            DeclareLaunchArgument(
                "simulator", default_value="false",
                description="Gazebo camera: TF only, images come from ros_gz_bridge",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
