#!/usr/bin/env bash
set -e
cd /home/mnt/ws
mkdir -p src

# Source ROS Noetic
source /opt/ros/noetic/setup.bash

# Build using catkin
catkin_make

# Source the workspace for future shells
echo "source /home/mnt/ws/devel/setup.bash" >> ~/.bashrc

# Source it now
source /home/mnt/ws/devel/setup.bash
