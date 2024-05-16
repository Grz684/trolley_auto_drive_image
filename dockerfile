FROM osrf/ros:foxy-desktop
SHELL ["/bin/bash","-c"]
COPY code /code/
WORKDIR /code
ENV DEBIAN_FRONTEND noninteractive
ENV DISPLAY=:1
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list
RUN source /opt/ros/foxy/setup.bash && \
apt update && \
apt install -y apt-utils && \
apt install -y fonts-wqy-zenhei && \
apt install -y ros-foxy-sensor-msgs-py && \
rosdep install --from-paths src -y --ignore-src && \
colcon build --packages-select lidar_msgs ros2_lidar linear_sensor_msgs && \
colcon build --symlink-install --packages-select lidar_data_handler linear_displacement_sensor
EXPOSE 2368/udp 2369/udp
STOPSIGNAL SIGINT
ENTRYPOINT source install/setup.bash && /usr/bin/python3 /opt/ros/foxy/bin/ros2 launch lidar_data_handler trolley_auto_drive_launch.py
