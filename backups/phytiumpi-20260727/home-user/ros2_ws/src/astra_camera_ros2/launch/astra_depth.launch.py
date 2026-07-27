from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="astra_camera_ros2",
                executable="depth_camera_node",
                name="astra_depth_camera",
                output="screen",
                parameters=[{"frame_id": "camera_depth_optical_frame"}],
            )
        ]
    )
