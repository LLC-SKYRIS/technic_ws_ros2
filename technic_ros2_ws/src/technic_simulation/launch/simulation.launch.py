import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def _validate_px4(context):
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    binary = os.path.join(px4_dir, "build", "px4_sitl_default", "bin", "px4")
    if not os.path.isfile(binary):
        raise RuntimeError(
            f"PX4 SITL binary not found: {binary}. "
            "Build PX4 v1.17 first with: make px4_sitl_default"
        )
    return []


def generate_launch_description():
    simulation_share = get_package_share_directory("technic_simulation")
    description_share = get_package_share_directory("technic_description")
    bringup_share = get_package_share_directory("technic_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world = os.path.join(simulation_share, "worlds", "technic.sdf")
    model_path = os.path.join(description_share, "models")
    flight_config = os.path.join(bringup_share, "config", "flight.yaml")

    px4_dir = LaunchConfiguration("px4_dir")
    gui = LaunchConfiguration("gui")
    start_agent = LaunchConfiguration("start_agent")
    spawn_pose = LaunchConfiguration("spawn_pose")

    px4_models = PathJoinSubstitution(
        [px4_dir, "Tools", "simulation", "gz", "models"]
    )
    px4_plugins = PathJoinSubstitution(
        [
            px4_dir,
            "build",
            "px4_sitl_default",
            "src",
            "modules",
            "simulation",
            "gz_plugins",
        ]
    )
    px4_binary = PathJoinSubstitution(
        [px4_dir, "build", "px4_sitl_default", "bin", "px4"]
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -s {world}"}.items(),
        condition=UnlessCondition(gui),
    )

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        condition=IfCondition(start_agent),
    )

    px4 = ExecuteProcess(
        cmd=[px4_binary],
        cwd=px4_dir,
        output="screen",
        emulate_tty=True,
        additional_env={
            "PX4_GZ_STANDALONE": "1",
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "gz_technic",
            "PX4_GZ_WORLD": "technic",
            "PX4_GZ_MODEL_POSE": spawn_pose,
        },
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/main_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            (
                "/main_camera/image_raw/camera_info"
                "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ),
            "/rangefinder@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
    )

    flight_manager = Node(
        package="technic_flight",
        executable="flight_manager",
        name="flight_manager",
        output="screen",
        parameters=[flight_config, {"use_sim_time": True}],
        emulate_tty=True,
    )

    bringup_launch = os.path.join(get_package_share_directory("technic_bringup"), "launch")
    main_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch, "main_camera.launch.py")),
        launch_arguments={
            "simulator": "true",
            "direction_z": "down",
            "direction_y": "backward",
        }.items(),
    )
    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("technic_aruco"), "launch", "aruco.launch.py")
        ),
        launch_arguments={
            "placement": "floor",
            "map": LaunchConfiguration("aruco_map_file"),
            "camera_info_topic": "/main_camera/image_raw/camera_info",
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("aruco")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "aruco",
                default_value="false",
                description="Start ArUco detection, map -> aruco_map TF and VPE",
            ),
            DeclareLaunchArgument(
                "aruco_map_file", default_value="map1.txt", description="technic_aruco map"
            ),
            DeclareLaunchArgument(
                "px4_dir",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "PX4-Autopilot"]
                ),
                description="PX4-Autopilot v1.17 checkout",
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Start Gazebo GUI"
            ),
            DeclareLaunchArgument(
                "start_agent",
                default_value="true",
                description="Start Micro XRCE-DDS Agent on UDP port 8888",
            ),
            DeclareLaunchArgument(
                "spawn_pose",
                default_value="0,0,0.3,0,0,0",
                description="PX4_GZ_MODEL_POSE value",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    model_path,
                    os.pathsep,
                    px4_models,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                [
                    px4_plugins,
                    os.pathsep,
                    EnvironmentVariable(
                        "GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""
                    ),
                ],
            ),
            OpaqueFunction(function=_validate_px4),
            gazebo_gui,
            gazebo_headless,
            agent,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=px4,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="PX4 SITL exited; stopping simulation"
                            )
                        )
                    ],
                )
            ),
            px4,
            bridge,
            flight_manager,
            main_camera,
            aruco,
        ]
    )
