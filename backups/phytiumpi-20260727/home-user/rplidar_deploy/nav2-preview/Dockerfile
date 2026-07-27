FROM ros:humble-ros-base

RUN sed -i \
      -e 's|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g' \
      -e 's|Types: deb deb-src|Types: deb|' \
      /usr/share/ros-apt-source/ros2.sources \
    && curl -fsSL -o /tmp/uuid-dev.deb \
      https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/main/u/util-linux/uuid-dev_2.37.2-4ubuntu3.5_arm64.deb \
    && curl -fsSL -o /tmp/libboost-program-options.deb \
      https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/main/b/boost1.74/libboost-program-options1.74.0_1.74.0-14ubuntu3_arm64.deb \
    && dpkg -i /tmp/uuid-dev.deb /tmp/libboost-program-options.deb \
    && mv /etc/apt/sources.list /etc/apt/sources.list.disabled \
    && apt-get -o Acquire::ForceIPv4=true update \
    && apt-get -o Acquire::ForceIPv4=true install -y --no-install-recommends \
      ros-humble-nav2-planner \
      ros-humble-nav2-navfn-planner \
      ros-humble-nav2-lifecycle-manager \
    && mv /etc/apt/sources.list.disabled /etc/apt/sources.list \
    && rm -rf /var/lib/apt/lists/* /tmp/*.deb

COPY planner_bridge.py /opt/nav2-preview/planner_bridge.py
COPY start-nav2-preview.sh /opt/nav2-preview/start-nav2-preview.sh
RUN chmod 0755 /opt/nav2-preview/planner_bridge.py /opt/nav2-preview/start-nav2-preview.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/opt/nav2-preview/start-nav2-preview.sh"]
