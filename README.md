# Duckiebot Simulation (ROS 2 Humble)


## Workspace Setup

Start the container in VS Code:

```bash
Ctrl+Shift+P -> Dev Containers: Rebuild and Reopen in Container
```
<p>
The devcontainer builds a ROS 2 Humble environment and runs:

```bash
cd /home/mnt/ws
colcon build --symlink-install
```

For a manual rebuild inside the container:

```bash
cd /home/mnt/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

Launch the Duckiebot world in Gazebo Classic:

```bash
ros2 launch duckiebot_gazebo duckie_world.launch.py
```

Launch with keyboard teleop in the same launch file:

```bash
ros2 launch duckiebot_gazebo duckie_world.launch.py use_teleop:=true
```

Or run teleop separately in another terminal:

```bash
source /home/mnt/ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

View the robot in RViz:

```bash
ros2 launch duckiebot_description view_duckiebot.launch.py
```

Spawn only the robot into an already-running Gazebo instance:

```bash
ros2 launch duckiebot_gazebo duckiebot_gazebo.launch.py
```

Useful topics after launching:

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /duckiebot/imu/data
ros2 topic echo /duckiebot/camera1/image_raw
```
