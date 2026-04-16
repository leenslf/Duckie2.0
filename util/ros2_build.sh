#!/usr/bin/env bash
cd /home/mnt/workspace
colcon build
echo 'source /home/mnt/workspace/install/setup.bash' >> /root/.bashrc
source install/setup.bash