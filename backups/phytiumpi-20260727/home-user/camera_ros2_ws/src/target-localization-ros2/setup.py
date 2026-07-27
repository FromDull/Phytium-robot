from setuptools import find_packages, setup


package_name = "target_localization_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Phytium Robot Team",
    maintainer_email="robot@localhost",
    description="Fuse 2D detections with aligned depth and publish stable 3D targets.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_localizer = target_localization_ros2.target_localizer:main",
        ],
    },
)
