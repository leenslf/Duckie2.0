# Duckie2.0 ROS 2 Workspace

This workspace is isolated from the existing catkin workspace under `ws/`.

Current migration scope:

- `duckiebot_description` ported to `ament_cmake`
- `duckiebot_gazebo` ported to `ament_cmake`
- Gazebo Classic launch moved to ROS 2 Python launch
- Gazebo plugin XML updated to the ROS 2 `gazebo_ros` parameter schema

Expected workflow once ROS 2 and Gazebo Classic dependencies are installed:

```bash
cd /home/leen/Duckie2.0/ros2_ws
colcon build
source install/setup.bash
ros2 launch duckiebot_gazebo duckie_world.launch.py
```

Optional keyboard teleop:

```bash
ros2 launch duckiebot_gazebo duckie_world.launch.py teleop:=true
```
