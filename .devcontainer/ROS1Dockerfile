FROM osrf/ros:noetic-desktop-full

ARG USERNAME=grad-project
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

ENV SHELL=/bin/bash \
    ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    sudo \
    autoconf \
    git \
    libargon2-dev \
    libssl-dev \
    libx11-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libxext-dev \
    lsb-release \
    gnupg \
    flex \
    bison \
    gstreamer1.0-pulseaudio \
    python3-pip \
    nano \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-localization \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    && rm -rf /var/lib/apt/lists/*


RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    usermod -a -G plugdev $USERNAME

USER $USERNAME

RUN pip install --user --upgrade numpy==1.24.4

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
