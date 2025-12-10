To see the models in the insert tab in gazebo run this line before running gazebo
```
export GAZEBO_MODEL_PATH=/home/mnt/ws/src/duckiebot_gazebo/models:$GAZEBO_MODEL_PATH
```

## Workspace Setup

- **Start the container:**

    In VsCode ctrl + alt + p -> Rebuild and Reopen in container 
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
