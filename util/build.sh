#!/usr/bin/env bash
set -e
cd /home/mnt/ws
mkdir -p src

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Remove stale ROS 1/catkin outputs before the ROS 2 build.
rm -rf build devel install log .catkin_workspace

# Build using colcon
colcon build --symlink-install

# Source the workspace for future shells
grep -qxF "source /home/mnt/ws/install/setup.bash" ~/.bashrc || \
  echo "source /home/mnt/ws/install/setup.bash" >> ~/.bashrc
