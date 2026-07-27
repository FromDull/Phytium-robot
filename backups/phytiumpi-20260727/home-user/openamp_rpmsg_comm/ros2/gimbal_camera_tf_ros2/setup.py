from setuptools import find_packages, setup


package_name = "gimbal_camera_tf_ros2"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gimbal_tf_calibration.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cnvhk",
    maintainer_email="cnvhk@todo.todo",
    description="Validated dynamic gimbal camera TF publisher",
    license="MIT",
    entry_points={
        "console_scripts": ["gimbal_camera_tf = gimbal_camera_tf_ros2.node:main"]
    },
)
