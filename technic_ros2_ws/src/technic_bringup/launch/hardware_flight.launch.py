from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

# rmw_cyclonedds with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST gives each process a
# participant index and allows only 32 of them; one launch is ~25 processes, so a
# second launch (or leftovers of a killed one) makes nodes fail with "Failed to find
# a free participant index". The same value must be set in every shell (README).
CYCLONEDDS_URI_DEFAULT = (
    '<CycloneDDS><Domain id="any"><Discovery>'
    '<MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>'
    '</Discovery></Domain></CycloneDDS>'
)


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _aruco(context):
    # As technic.launch: include if aruco or force_init; force_init alone runs
    # only the zero pose publisher.
    aruco = _as_bool(context, "aruco")
    force_init = _as_bool(context, "force_init")
    if not (aruco or force_init):
        return []
    aruco_launch = os.path.join(
        get_package_share_directory("technic_aruco"), "launch", "aruco.launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_launch),
            launch_arguments={
                "placement": LaunchConfiguration("aruco_placement"),
                "map": LaunchConfiguration("aruco_map_file"),
                "length": LaunchConfiguration("aruco_length"),
                "force_init": str(force_init).lower(),
                "disable": str(not aruco).lower(),
            }.items(),
        )
    ]


GCS_URLS = {
    # As technic/launch/mavros.launch in ROS 1.
    "tcp": "tcp-l://0.0.0.0:5760",
    "udp": "udp://0.0.0.0:14550@14550",
    "udp-b": "udp-b://:14550@14550",
    "none": "",
}


# mavros distance_sensor subscriber "rangefinder_sub" (config/mavros.yaml).
RANGEFINDER_MAVROS_TOPIC = "/mavros/mavros/rangefinder_sub"
# mavros px4flow plugin input, sent to PX4 as OPTICAL_FLOW_RAD.
PX4FLOW_MAVROS_TOPIC = "/mavros/mavros/px4flow/raw/send"


def _mavros(context):
    if not _as_bool(context, "mavros"):
        return []
    gcs_bridge = LaunchConfiguration("gcs_bridge").perform(context)
    if gcs_bridge not in GCS_URLS:
        raise RuntimeError(f"gcs_bridge must be one of {', '.join(GCS_URLS)}")
    fcu_url = "{}:{}".format(
        LaunchConfiguration("serial_device").perform(context),
        LaunchConfiguration("baudrate").perform(context),
    )
    mavros_config = os.path.join(
        get_package_share_directory("technic_bringup"), "config", "mavros.yaml"
    )
    return [
        Node(
            package="mavros",
            executable="mavros_node",
            namespace="mavros",
            output="screen",
            parameters=[
                mavros_config,
                {"fcu_url": fcu_url, "gcs_url": GCS_URLS[gcs_bridge]},
            ],
            respawn=True,
            respawn_delay=1.0,
        )
    ]


def _rangefinder(context):
    # As technic.launch in ROS 1: vl53l1x driver, base_link -> rangefinder TF and
    # relay of /rangefinder/range to the mavros distance_sensor subscriber.
    if not _as_bool(context, "rangefinder_vl53l1x"):
        return []
    vl53l1x_launch = os.path.join(
        get_package_share_directory("vl53l1x"), "launch", "vl53l1x.launch.py"
    )
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(vl53l1x_launch)),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rangefinder_frame",
            arguments=[
                "--z", "-0.05", "--pitch", "1.5707963268",
                "--frame-id", "base_link", "--child-frame-id", "rangefinder",
            ],
        ),
        Node(
            package="topic_tools",
            executable="relay",
            name="range_to_mavros",
            arguments=["/rangefinder/range", RANGEFINDER_MAVROS_TOPIC],
            output="screen",
            respawn=True,
            condition=IfCondition(LaunchConfiguration("mavros")),
        ),
    ]


def _optical_flow(context):
    # As technic.launch in ROS 1 (there on the down-facing second camera; here the
    # main camera looks down). PX4 fuses it according to EKF2_OF_CTRL.
    if not (_as_bool(context, "optical_flow") and _as_bool(context, "main_camera")):
        return []
    return [
        Node(
            package="technic_optical_flow",
            executable="optical_flow",
            name="optical_flow",
            output="screen",
            respawn=True,
            remappings=[
                ("image_raw", "/main_camera/image_raw"),
                ("camera_info", "/main_camera/camera_info"),
                ("vision_pose", "/vision_pose/pose"),
            ],
            parameters=[
                {
                    "flow_topic": PX4FLOW_MAVROS_TOPIC,
                    "calc_flow_gyro": True,
                    "roi_rad": 0.8,
                    # Keep publishing flow while the ArUco VPE is available; PX4
                    # decides which estimators to fuse through EKF2 parameters.
                    "disable_on_vpe": False,
                }
            ],
        )
    ]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("technic_bringup"), "config", "flight.yaml"
    )
    enable_flight = LaunchConfiguration("enable_flight")
    launch_dir = os.path.join(get_package_share_directory("technic_bringup"), "launch")

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
                "mavros",
                default_value="true",
                description="Start mavros: PX4 MAVLink on serial_device for ROS and QGC",
            ),
            DeclareLaunchArgument(
                "enable_flight",
                default_value="false",
                description="Explicit opt-in required to expose hardware flight commands",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value="/dev/ttyS0",
                description="UART connected to the PX4 MAVLink port",
            ),
            DeclareLaunchArgument("baudrate", default_value="921600"),
            DeclareLaunchArgument(
                "gcs_bridge",
                default_value="tcp",
                description="QGroundControl link through mavros: tcp (port 5760), "
                "udp (port 14550), udp-b (broadcast) or none",
            ),
            DeclareLaunchArgument("main_camera", default_value="true"),
            # Camera looks down at floor markers (main_camera.launch.py arguments).
            DeclareLaunchArgument("direction_z", default_value="down"),
            DeclareLaunchArgument("direction_y", default_value="backward"),
            DeclareLaunchArgument(
                "optical_flow",
                default_value="true",
                description="optical flow from the down-facing main camera to PX4",
            ),
            DeclareLaunchArgument(
                "rangefinder_vl53l1x",
                default_value="true",
                description="VL53L1X rangefinder on I2C bus 1, sent to PX4 through mavros",
            ),
            DeclareLaunchArgument(
                "aruco",
                default_value="true",
                description="ArUco map -> aruco_map TF and VPE; base_link TF and "
                "forwarding the vision pose to PX4 need the flight manager",
            ),
            DeclareLaunchArgument("aruco_placement", default_value="floor"),
            DeclareLaunchArgument("aruco_map_file", default_value="map1.txt"),
            DeclareLaunchArgument("aruco_length", default_value="0.22"),
            DeclareLaunchArgument(
                "force_init",
                default_value="false",
                description="force estimator to init by publishing zero pose",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "main_camera.launch.py")),
                condition=IfCondition(LaunchConfiguration("main_camera")),
            ),
            DeclareLaunchArgument(
                "web",
                default_value="true",
                description="Technic web UI: pages on :8000/technic/, rosbridge :9090, "
                "web_video_server :8080 (technic_web/launch/web.launch.py)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("technic_web"), "launch", "web.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("web")),
            ),
            OpaqueFunction(function=_aruco),
            OpaqueFunction(function=_mavros),
            OpaqueFunction(function=_rangefinder),
            OpaqueFunction(function=_optical_flow),
            Node(
                package="technic_flight",
                executable="flight_manager",
                name="flight_manager",
                output="screen",
                parameters=[config, {"autopilot.backend": "mavros"}],
                emulate_tty=True,
                condition=IfCondition(enable_flight),
            ),
        ]
    )
