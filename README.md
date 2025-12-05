# Duckiebot Simulation (ROS Noetic)

This package provides a lightweight Duckiebot model for Gazebo simulation.  
The robot includes:

- Differential drive wheels (Gazebo diff-drive plugin)
- Camera sensor (Gazebo ROS camera plugin)
- IMU sensor (Gazebo ROS IMU plugin)
- Simple URDF with meshes, joints, and inertials



## Workspace Setup

- **Start the container:**

    In VsCode `Ctrl+Shift+P` -> Rebuild and Reopen in container 
- Build
    ```bash
    cd ~/ws
    catkin_make
    ```
- Source 
    ``` bash 
    source devel/setup.bash
    (Optional) Add sourcing to your .bashrc:
    ```

## Launch
To launch sim with teleop keyboard: `roslaunch duckiebot_gazebo duckie_world.launch`.
Checkout the other launch files for more. 
