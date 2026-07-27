FROM ros:humble-ros-base

RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.aliyun.com/ubuntu-ports|g' /etc/apt/sources.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

COPY slam_toolbox.yaml /opt/slam/config/slam_toolbox.yaml

CMD ["bash"]
