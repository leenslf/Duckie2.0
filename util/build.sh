#!/usr/bin/env bash
cd /home/mnt/ws
colcon build
echo 'source /home/mnt/ws/install/setup.bash' >> /root/.bashrc
source install/setup.bash