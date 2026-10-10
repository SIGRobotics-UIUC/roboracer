# ROS 2 Humble + the AutoDRIVE bridge. ros:humble is multi-arch (amd64 + arm64), so this
# builds natively on Intel/AMD Linux, Windows (WSL2) and Apple Silicon Macs.
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    git vim \
    ros-humble-ackermann-msgs \
    ros-humble-tf2-ros \
    ros-humble-foxglove-bridge \
    ros-humble-tf-transformations \
    python3-pil \
    ros-humble-cv-bridge \
    # gevent from apt: pip can't build it on every platform
    python3-gevent python3-gevent-websocket \
    && rm -rf /var/lib/apt/lists/*

# AutoDRIVE's Socket.IO client needs these exact versions
RUN pip3 install --no-cache-dir python-socketio==4.2.0 python-engineio==3.13.0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "[ -f /root/ws/install/setup.bash ] && source /root/ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/ws
CMD ["bash"]
