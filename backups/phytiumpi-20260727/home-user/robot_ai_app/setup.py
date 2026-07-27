from glob import glob
import os

from setuptools import find_packages, setup


package_name = "robot_ai_app"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cnvhk",
    maintainer_email="cnvhk@todo.todo",
    description="ROS 2 native AI control layer for the Webots wheel-leg robot",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "robot_ai_agent = robot_ai_app.ros_agent_loop:main",
            "robot_ai_camera_dump = robot_ai_app.ros_camera_dump:main",
            "robot_ai_chat = robot_ai_app.ros_chat_control:main",
            "robot_ai_teleop = robot_ai_app.ros_teleop:main",
            "robot_ai_voice_demo = robot_ai_app.voice_chat_demo:main",
        ],
    },
)
