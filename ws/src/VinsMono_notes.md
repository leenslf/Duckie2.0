### Install this test data after you install vins mono and build (and source ofc)
```bash
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
``` 
### Terminal 1
```bash
roslaunch vins_estimator euroc.launch
```
### Terminal 2  
```bash
roslaunch vins_estimator vins_rviz.launch
```
### Terminal 3
```bash
rosbag play MH_01_easy.bag
```
### You will observe the following topics
``` bash
$ rostopic list 
/cam0/image_raw
/cam1/image_raw
/clock
/feature_tracker/feature
/feature_tracker/feature_img
/feature_tracker/restart
/imu0
/leica/position
/pose_graph/base_path
/pose_graph/camera_pose_visual
/pose_graph/key_odometrys
/pose_graph/match_image
/pose_graph/match_points
/pose_graph/no_loop_path
/pose_graph/path_1
/pose_graph/path_2
/pose_graph/path_3
/pose_graph/path_4
/pose_graph/path_5
/pose_graph/path_6
/pose_graph/path_7
/pose_graph/path_8
/pose_graph/path_9
/pose_graph/pose_graph
/pose_graph/pose_graph_path
/rosout
/rosout_agg
/tf
/vins_estimator/camera_pose
/vins_estimator/camera_pose_visual
/vins_estimator/extrinsic
/vins_estimator/history_cloud
/vins_estimator/imu_propagate
/vins_estimator/key_poses
/vins_estimator/keyframe_point
/vins_estimator/keyframe_pose
/vins_estimator/odometry
/vins_estimator/path
/vins_estimator/point_cloud
/vins_estimator/relo_relative_pose
/vins_estimator/relocalization_path

``` 
In this case we are launching 3 nodes: `feature_tracker`, `vins_estimator` and `pose_graph`.
```
feature_tracker  →  vins_estimator  →  pose_graph
     (vision)         (core VIO)      (loop closure)
```

**`feature_tracker` Node**

- Subscribes to: raw camera images (/duckiebot/camera1/image_raw)
- Processes:

    Detects features (corner points) in images
    Tracks features across consecutive frames using optical flow
    Rejects outliers with RANSAC


- Publishes:

    /feature_tracker/feature - Tracked feature points (normalized coordinates)
    /feature_tracker/img_track - Debug image showing tracked features

**`vins_estimator` Node**

- Subscribes to:

    /feature_tracker/feature - Features from feature_tracker
    /imu_data -  IMU data


- Processes:

    IMU preintegration
    Initialization
    The core VIO optimization (sliding window)
    State estimation


- Publishes:

    /vins_estimator/odometry -  main output!
    /vins_estimator/imu_propagate - High-rate pose
    /vins_estimator/point_cloud - 3D feature points
    /vins_estimator/keyframe_pose - For loop closure

**`pose_graph` Node**
- Subscribes to:

    /vins_estimator/keyframe_pose - Keyframes from VIO
    /vins_estimator/keyframe_point - Feature descriptors
    /vins_estimator/extrinsic - Camera-IMU calibration


- Processes:

    Loop detection (have we been here before?)
    Place recognition using DBoW2
    Global pose graph optimization (4-DOF)
    Drift correction


- Publishes:

    /pose_graph/match_image - Loop closure matches
    /pose_graph/pose_graph - Global trajectory






#

Our main focus right now is **`vins_estimator` Node**. Checkout config and launch under that package, We will use the calibration params that @Danial will find in  `/VINS-Mono/config/duckiebot/duckiebot_config.yaml` (currently we use approximate values).