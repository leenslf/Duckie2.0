This ROS 2 package contains the Gazebo Classic launch files and world assets for the Duckiebot simulation.

Use the ROS 2 launch entrypoint directly:

```bash
cd /home/leen/Duckie2.0/ros2_ws
colcon build
source install/setup.bash
ros2 launch duckiebot_gazebo duckie_world.launch.py
```

If you want keyboard teleoperation and `xterm` is installed, enable it explicitly:

```bash
ros2 launch duckiebot_gazebo duckie_world.launch.py teleop:=true
```

There is also a stripped-down test model that uses a very simple xacro:

```bash
ros2 launch duckiebot_gazebo simple_duckiebot.launch.py
```
