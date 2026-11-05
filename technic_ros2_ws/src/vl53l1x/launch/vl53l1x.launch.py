from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Same settings as the rangefinder node in ROS 1 technic.launch.
    return LaunchDescription(
        [
            Node(
                package="vl53l1x",
                executable="vl53l1x_node",
                name="rangefinder",
                output="screen",
                parameters=[
                    {
                        "frame_id": "rangefinder",
                        "i2c_bus": 1,
                        "min_signal": 0.4,
                        "pass_statuses": [0, 6, 7, 11],
                    }
                ],
            )
        ]
    )
