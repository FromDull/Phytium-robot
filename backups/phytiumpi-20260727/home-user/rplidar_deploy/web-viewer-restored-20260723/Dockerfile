FROM lidar-web-viewer:humble

RUN sed -i 's|https://mirrors.aliyun.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends python3-pil \
    && rm -rf /var/lib/apt/lists/*

COPY server.py index.html semantic_map.py three.min.js /opt/lidar_web_viewer/

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /opt/lidar_web_viewer/server.py"]
