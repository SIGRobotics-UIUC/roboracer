# ROS 2 Humble + the AutoDRIVE bridge. ros:humble is multi-arch (amd64 + arm64), so this
# builds natively on Intel/AMD Linux, Windows (WSL2) and Apple Silicon Macs.
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    git vim \
    ros-humble-ackermann-msgs \
    ros-humble-rosbag2-py \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rqt \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-visual-tools \
    ros-humble-rqt-common-plugins \
    ros-humble-tf2-ros \
    ros-humble-foxglove-bridge \
    ros-humble-tf-transformations \
    python3-pil \
    # fast_bridge front camera (publish_camera:=true)
    ros-humble-cv-bridge \
    # perception: Cartographer SLAM, robot_localization for the experimental IMU EKF
    ros-humble-cartographer-ros \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    # gevent from apt: pip can't build it on every platform
    python3-gevent python3-gevent-websocket \
    # matplotlib WebAgg backend (control/tools/lidar_plot.py)
    python3-tornado \
    && rm -rf /var/lib/apt/lists/*

# AutoDRIVE's Socket.IO client needs these exact versions
RUN pip3 install --no-cache-dir python-socketio==4.2.0 python-engineio==3.13.0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "[ -f /root/ws/install/setup.bash ] && source /root/ws/install/setup.bash" >> ~/.bashrc

# Planning: apt deps + TUM mintime optimizer in its own venv (/opt/tum_venv), see the script
COPY ws/src/planning/docker /tmp/planning_docker
RUN bash /tmp/planning_docker/install_deps.sh && rm -rf /tmp/planning_docker

WORKDIR /root/ws
CMD ["bash"]
