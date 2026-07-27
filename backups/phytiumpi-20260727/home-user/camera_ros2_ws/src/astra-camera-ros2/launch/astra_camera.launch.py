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
                        "width": 320,
                        "height": 240,
                        "fps": 30,
                        "publish_every_n_frames": 3,
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
                        "width": 320,
                        "height": 240,
                        "fps": 30,
                        "publish_every_n_frames": 2,
                        "camera_matrix": [
                            301.7490836986282,
                            0.0,
                            155.88566585359035,
                            0.0,
                            302.8299897196495,
                            125.2979044892051,
                            0.0,
                            0.0,
                            1.0,
                        ],
                        "distortion_coefficients": [
                            0.0188221408982716,
                            0.3978707903457919,
                            0.003406344566401516,
                            -0.006050046675947736,
                            0.0,
                        ],
                        "projection_matrix": [
                            301.7490836986282,
                            0.0,
                            155.88566585359035,
                            0.0,
                            0.0,
                            302.8299897196495,
                            125.2979044892051,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                        ],
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="astra_depth_to_color_tf",
                arguments=[
                    "--x", "0.007019209010330862",
                    "--y", "-0.031575880493780475",
                    "--z", "0.022703656281993386",
                    "--qx", "-0.00332550152307356",
                    "--qy", "-0.00907323598714789",
                    "--qz", "-0.0042639132235729625",
                    "--qw", "0.9999442166802923",
                    "--frame-id", "camera_color_optical_frame",
                    "--child-frame-id", "camera_depth_optical_frame",
                ],
            ),
            Node(
                package="astra_camera_ros2",
                executable="depth_registration_node",
                name="astra_depth_registration",
                output="screen",
                parameters=[
                    {
                        "depth_intrinsics": [
                            285.17110450338835,
                            285.1711090021791,
                            159.5,
                            119.5,
                        ],
                    }
                ],
            ),
        ]
    )
