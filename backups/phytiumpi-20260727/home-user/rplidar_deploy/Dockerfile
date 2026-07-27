FROM rplidar-ros2:humble

COPY server.py index.html /opt/lidar_web_viewer/

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /opt/lidar_web_viewer/server.py"]
