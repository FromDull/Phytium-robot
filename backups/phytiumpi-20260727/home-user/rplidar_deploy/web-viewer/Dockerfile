FROM ros:humble-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pil \
    && rm -rf /var/lib/apt/lists/*

COPY server.py semantic_map.py depth_voxel_map.py index.html web_logic.js three.min.js /opt/lidar_web_viewer/

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /opt/lidar_web_viewer/server.py"]
