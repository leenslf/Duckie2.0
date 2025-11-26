---

# Duckiebot Simulation Startup Guide

## Overview

This guide explains how to start the Duckiebot simulation in Gazebo using ROS Noetic inside a Docker container. It includes notes about multiple terminals, sourcing, `roscore`, and spawning the robot manually.

---

## 1️⃣ Prerequisites

* Docker container already built (`duckiebot_sim`) and running ROS Noetic.
* Duckiebot workspace mounted in container: `~/duckiebot_ws/src`
* All ROS packages are compiled (`catkin_make` or `catkin build`)
* Teleoperation package installed: `teleop_twist_keyboard`
* xacro installed: `ros-noetic-xacro`

---

## 2️⃣ Terminal Setup

It is recommended to use **multiple terminal windows** (or `tmux`) for different processes:

| Terminal | Purpose                     |
| -------- | --------------------------- |
| 1        | `roscore`                   |
| 2        | Gazebo server / empty world |
| 3        | Robot spawn & controller    |
| 4        | Teleoperation (drive robot) |

> **Important:** Every terminal inside the container must source your workspace and ROS environment:
>
> ```bash
> source /opt/ros/noetic/setup.bash
> source ~/duckiebot_ws/devel/setup.bash
> ```

---

## 3️⃣ Start the Simulation

### Step 1: Enter the Docker container

```bash
docker start -ai duckiebot_sim
```

or if already running:

```bash
docker exec -it duckiebot_sim bash
```

---

### Step 2: Terminal 1 — Start ROS core

```bash
source /opt/ros/noetic/setup.bash
source ~/duckiebot_ws/devel/setup.bash
roscore
```

---

### Step 3: Terminal 2 — Launch Gazebo

```bash
source /opt/ros/noetic/setup.bash
source ~/duckiebot_ws/devel/setup.bash

roslaunch gazebo_ros empty_world.launch
```

> This starts Gazebo with an empty world. The window may appear on your host if X11 forwarding is set.

---

### Step 4: Terminal 3 — Spawn the Duckiebot

1. **Source environment**:

```bash
source /opt/ros/noetic/setup.bash
source ~/duckiebot_ws/devel/setup.bash
```

2. **Generate URDF from xacro**:

```bash
xacro ~/duckiebot_ws/src/duckiebot_description/urdf/duckiebot.xacro > /tmp/duckiebot.urdf
```

> Replace the path if your URDF/xacro is located elsewhere.

3. **Set robot description**:

```bash
rosparam set /robot_description "$(cat /tmp/duckiebot.urdf)"
```

4. **Spawn the robot in Gazebo**:

```bash
rosrun gazebo_ros spawn_model -param robot_description -urdf -model duckiebot -x 0 -y 0 -z 0.2
```

> This command places the Duckiebot in the world at coordinates `(0,0,0.2)`.

5. **Start controllers**:

```bash
rosrun controller_manager spawner joint_state_controller diff_drive_controller
```

> Verify controllers are running:

```bash
rosservice list | grep controller
```

---

### Step 5: Terminal 4 — Teleoperation

```bash
source /opt/ros/noetic/setup.bash
source ~/duckiebot_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

* `i` → forward
* `k` → stop
* `j` → turn left
* `l` → turn right
* `u/o` → forward + turn

> Use `q/z` to increase/decrease maximum speed.

---

## 4️⃣ Notes & Tips

* **Sourcing:** Always source both ROS and workspace in each new terminal.
* **Terminal management:** You can use `tmux` or `screen` to split windows and keep multiple nodes running.
* **ROS parameters:** If `spawn_model` fails with `KeyError: robot_description`, the parameter was not set correctly. Make sure `rosparam set /robot_description ...` succeeds.
* **Gazebo display issues:** Use `--env="DISPLAY=$DISPLAY"` and mount `/tmp/.X11-unix` when starting Docker.
* **Persistent workspace:** Mount your host workspace inside Docker so your files persist across container restarts.

---

✅ Following this sequence ensures the robot is spawned, controllers are loaded, and you can drive it with your keyboard.

---
