from glob import glob
from setuptools import find_packages, setup


package_name = "chassis_control_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CNVHK",
    maintainer_email="cnvhk@localhost",
    description="ROS 2 bridge for OpenAMP chassis motion control",
    license="MIT",
    entry_points={
        "console_scripts": [
            "chassis_node = chassis_control_ros2.chassis_node:main",
        ],
    },
)
