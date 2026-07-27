from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    share = get_package_share_directory("chassis_control_ros2")
    config = os.path.join(share, "config", "chassis.yaml")
    return LaunchDescription([
        Node(
            package="chassis_control_ros2",
            executable="chassis_node",
            name="openamp_chassis",
            output="screen",
            parameters=[config],
        )
    ])
