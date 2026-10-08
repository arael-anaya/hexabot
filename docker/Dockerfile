# Hexapod ROS 2 development image.
# ROS 2 Humble (Ubuntu 22.04) matches the ROS 2 bridge bundled with Isaac Sim.
# Isaac Sim lives in its own container (see compose.isaac.yaml) so this image
# can be rebuilt freely.
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    # Python dev
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-transforms3d \
    # Robot description / visualization
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-urdf-launch \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    # Control (joint trajectories for the 18 leg joints)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-position-controllers \
    # Isaac Sim interop: topic_tools relay + CycloneDDS as an alternate RMW
    ros-humble-topic-tools \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-backward-ros \
    # Handy utilities
    mesa-utils \
    bash-completion \
    curl \
    nano \
    tmux \
    vim \
    wget \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# ── rosdep ────────────────────────────────────────────────────────────────────
RUN rosdep update

# ── Auto-source ROS and the local workspace on every shell ────────────────────
# The workspace overlay is sourced only once it has been built.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "[ -f /home/ros/ros_ws/install/setup.bash ] && source /home/ros/ros_ws/install/setup.bash" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc \
    && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

WORKDIR /home/ros/ros_ws

# ── Default shell entrypoint ──────────────────────────────────────────────────
CMD ["/bin/bash"]
