# mir_250_ros2
This is a ROS2 package for MiR 250 and UR5e series.

# Installation

## Preliminaries
## ROS2
If you haven't already installed [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your PC, you need to add the ROS2 apt repository.


## Source install

```bash
# create a ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

# clone mir_robot into the ros2 workspace
git clone https://github.com/giangalv/mir_robot_simulation.git

# use vcs to fetch linked repos
# $ sudo apt install python3-vcstool
vcs import < src/mir_robot/ros2.repos src --recursive

# use rosdep to install all dependencies (including ROS itself)
sudo apt update
sudo apt install -y python3-rosdep

sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-librealsense2*

rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# build all packages in the workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select ros2_laser_scan_merger
colcon build
```
You must source the workspace in each terminal you want to work in:
```
source ~/ros2_ws/install/setup.bash
```

#### Fix time synchronization manually

* In the Mir dashboard (mir.com in the Mir-Wifi), go to "Service" ->
  "Configuration" -> "System settings" -> "Time settings" -> "Set device time
  on robot"

Use **load from device** to sync with the system time!

#### Test the connection is working

This tests the connection if returning the result from the MIR the 'MIR_IP_ADDR' and
  'MIR_API_KEY' are correct.

```bash
  curl -X GET "http://130.251.13.90/api/v2.0.0/status" -H "Authorization: Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
```

#### **Fix time synchronization using ROS2:**

From the package `mir_restapi` a node called `mir_restapi_server` can be run,
which can execute a time sync REST API call from the driver's host machine to
the Mir's host.

* Launch the node with the API key and mir hostname's IP address

    ```bash
    ros2 run mir_restapi mir_restapi_server --ros-args -p mir_hostname:='MIR_IP_ADDR' -p mir_restapi_auth:='YOUR_API_KEY' #distributor
    ```

    MIR_IP_ADDR: 130.251.13.90 
    YOUR_API_KEY: Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==
    ```

* Call the time sync service from terminal by invoking

    ```bash
    ros2 service call /mir_sync_time std_srvs/Trigger
    ```

#### **After time sync**

Keep in mind, that the time sync causes the mir_bridge to freeze. Therefore online time syncs are not recommended.


# RVIZ demo test

```bash
### rviz:
ros2 launch mir_description mir_display_launch.py
```
This should work and you are going to see the robot spawned inside the RVIZ2


# Gazebo demo (mapping)

```bash
### gazebo:
ros2 launch mir_gazebo mir_gazebo_launch.py  world:=maze

### mapping (slam_toolbox)
ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml

### navigation (optional)
ros2 launch mir_navigation navigation.py use_sim_time:=true cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

# Gazebo demo (Navigation with existing map)

```bash
### gazebo
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz


### localization (existing map)
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true
```

### Navigation on MiR

### Option 1: Launching the modules separately

```bash
### driver:
ros2 launch mir_driver mir_launch.py

### localization (amcl)
ros2 launch mir_navigation amcl.py use_sim_time:=false map:={path to existing map}

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=false
```

### Option 2: Use combined launch file

```bash
### combined launch file:
ros2 launch mir_navigation mir_nav_launch.py map:={path to /name of existing map}
```
# Dual Laser Merger
## Acknowledgement
The 3d files for MiR 250 are from [DFKI](https://github.com/DFKI-NI/mir_robot).

## Example Demo using recorded bag file
This demo shows merging of laser scan data from 2 lidars.
```
ros2 launch dual_laser_merger demo_laser_merger.launch.py
```

## Requirements
1. Lidar 1 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /lidar1/scan
      Type: sensor_msgs/msg/LaserScan
   ```
2. Lidar 2 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /lidar2/scan
      Type: sensor_msgs/msg/LaserScan
   ```
3. TF from Lidar 1 (`laser_1`) and Lidar 2 (`laser_2`) to Target frame (`lsc_mount`)
  ```
  ~$ ros2 topic echo /tf_static 
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 564956753
        frame_id: lsc_mount
      child_frame_id: laser_1
      transform:
        translation:
          x: 0.321967
          y: 0.221817
          z: 0.0
        rotation:
          x: 0.3826834321814926
          y: 0.9238795325873352
          z: 3.9573888241688663e-14
          w: -9.553981776262265e-14
  
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 580013258
        frame_id: lsc_mount
      child_frame_id: laser_2
      transform:
        translation:
          x: -0.321967
          y: -0.221817
          z: 0.0
        rotation:
          x: -0.9238795324744832
          y: 0.38268343245394154
          z: -9.553981775095244e-14
          w: -3.957388826986303e-14
  ```
