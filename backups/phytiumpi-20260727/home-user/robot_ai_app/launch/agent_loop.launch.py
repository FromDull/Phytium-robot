from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("policy", default_value="reactive"),
        DeclareLaunchArgument("steps", default_value="20"),
        Node(
            package="robot_ai_app",
            executable="robot_ai_agent",
            name="robot_ai_agent",
            output="screen",
            arguments=[
                "--policy", LaunchConfiguration("policy"),
                "--steps", LaunchConfiguration("steps"),
            ],
        ),
    ])
