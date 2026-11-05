"""ArUco markers and markers map detection (port of technic/launch/aruco.launch).

TF produced with aruco_vpe:=true (default, as in ROS 1):
  main_camera_optical -> aruco_map_detected   (aruco_map, every detection)
  map -> aruco_map                            (vpe_publisher, static offset)
  aruco_map -> aruco_<id>                     (aruco_map, static map markers)
and the vision pose on /vision_pose/pose for the PX4 backend.

With aruco_vpe:=false aruco_map publishes main_camera_optical -> aruco_map
directly, so map -> aruco_map is resolved through base_link.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _nodes(context):
    aruco_detect = _as_bool(context, "aruco_detect")
    aruco_map = _as_bool(context, "aruco_map")
    aruco_vpe = _as_bool(context, "aruco_vpe")
    placement = LaunchConfiguration("placement").perform(context)
    length = float(LaunchConfiguration("length").perform(context))
    map_name = LaunchConfiguration("map").perform(context)
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    use_sim_time = _as_bool(context, "use_sim_time")
    force_init = _as_bool(context, "force_init")
    disable = _as_bool(context, "disable")

    if placement not in ("floor", "ceiling", "unknown"):
        raise RuntimeError("placement must be floor, ceiling or unknown")
    known_vertical = "map" if placement in ("floor", "ceiling") else ""
    flip_vertical = placement == "ceiling"

    map_path = map_name if os.path.isabs(map_name) else os.path.join(
        get_package_share_directory("technic_aruco"), "map", map_name
    )
    camera_remaps = [("image_raw", image_topic), ("camera_info", camera_info_topic)]
    nodes = []

    # aruco_detect: detect aruco markers, estimate poses
    if aruco_detect and not disable:
        nodes.append(
            Node(
                package="technic_aruco",
                executable="aruco_detect",
                name="aruco_detect",
                output="screen",
                respawn=True,
                remappings=camera_remaps + [("map_markers", "/aruco_map/map")],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "dictionary": 2,  # DICT_4X4_250
                        "estimate_poses": True,
                        "send_tf": True,
                        "use_map_markers": aruco_map,
                        "known_vertical": known_vertical,
                        "flip_vertical": flip_vertical,
                        "length": length,
                        "transform_timeout": 0.1,
                        "cornerRefinementMethod": 2,  # contour refinement
                        "minMarkerPerimeterRate": 0.075,  # 0.075 for 320x240, 0.0375 for 640x480
                        # length override example: "length_override.3": 0.1
                    }
                ],
            )
        )

    # aruco_map: estimate aruco map pose
    if aruco_map and not disable:
        nodes.append(
            Node(
                package="technic_aruco",
                executable="aruco_map",
                name="aruco_map",
                output="screen",
                respawn=True,
                remappings=camera_remaps + [("markers", "/aruco_detect/markers")],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map": map_path,
                        "known_vertical": known_vertical,
                        "flip_vertical": flip_vertical,
                        "image_axis": True,
                        "frame_id": "aruco_map_detected" if aruco_vpe else "aruco_map",
                        "markers.frame_id": "aruco_map",
                        "markers.child_frame_id_prefix": "aruco_",
                        "transform_timeout": 0.1,
                    }
                ],
            )
        )

    # vpe publisher from aruco markers
    if aruco_vpe or force_init:
        remappings = [
            ("/vpe_publisher/vpe", "/vision_pose/pose"),
            ("telemetry", "/flight/telemetry"),
        ]
        parameters = {
            "use_sim_time": use_sim_time,
            "force_init": force_init,
            "offset_frame_id": "aruco_map",
            "transform_timeout": 0.1,
        }
        if aruco_vpe:
            remappings.append(("/vpe_publisher/pose_cov", "/aruco_map/pose"))
            parameters["frame_id"] = "aruco_map_detected"
        nodes.append(
            Node(
                package="technic_aruco",
                executable="vpe_publisher",
                name="vpe_publisher",
                output="screen",
                remappings=remappings,
                parameters=[parameters],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("aruco_detect", default_value="true"),
            DeclareLaunchArgument("aruco_map", default_value="true"),
            DeclareLaunchArgument("aruco_vpe", default_value="true"),
            DeclareLaunchArgument(
                "placement",
                default_value="floor",
                description="markers placement: floor, ceiling, unknown",
            ),
            DeclareLaunchArgument(
                "length", default_value="0.22", description="not-in-map markers length, m"
            ),
            DeclareLaunchArgument(
                "map",
                default_value="map1.txt",
                description="markers map file name in technic_aruco/map or an absolute path",
            ),
            DeclareLaunchArgument("image_topic", default_value="/main_camera/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/main_camera/camera_info"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "force_init",
                default_value="false",
                description="force estimator to init by publishing zero pose",
            ),
            DeclareLaunchArgument(
                "disable", default_value="false", description="only force init"
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
