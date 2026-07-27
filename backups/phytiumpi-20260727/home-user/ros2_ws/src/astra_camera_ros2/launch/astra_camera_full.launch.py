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
                parameters=[
                    {
                        "frame_id": "camera_depth_optical_frame",
                        "publish_every_n_frames": 1,
                    }
                ],
            ),
            Node(
                package="astra_camera_ros2",
                executable="color_camera_node",
                name="astra_color_camera",
                output="screen",
                parameters=[
                    {
                        "device": "/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0",
                        "frame_id": "camera_color_optical_frame",
                        "width": 640,
                        "height": 480,
                        "fps": 30,
                        "publish_every_n_frames": 1,
                    }
                ],
            ),
            Node(
                package="astra_camera_ros2",
                executable="camera_capture_bridge.py",
                name="camera_capture_bridge",
                output="screen",
                parameters=[{"shared_dir": "/root/robot_data"}],
            ),
        ]
    )
