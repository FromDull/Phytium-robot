FROM rplidar-ros2:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       libeigen3-dev \
       libpcl-dev \
       ros-humble-laser-geometry \
       ros-humble-pcl-conversions \
       ros-humble-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

COPY Simple-2D-LiDAR-Odometry /opt/lidar_odom_ws/src/lidar_odometry

RUN source /opt/ros/humble/setup.bash \
    && cd /opt/lidar_odom_ws \
    && colcon build --symlink-install --packages-select lidar_odometry

CMD ["bash"]
