# HRI_UR10: Human-Robot Interaction with UR10

This repository contains an integrated ROS 2 system for controlling a UR10 robot, an Intel RealSense camera, and an Arduino-based sensor/LED interface. It features a comprehensive PyQt5-based GUI for launching and monitoring the entire system, performing hand-eye calibration, and executing automated robot sequences.

## Features

- **Integrated Launch GUI**: A central control center (`launch_gui.py`) to start/stop the UR10 driver, RealSense camera, Arduino bridge, Hand-eye calibration, and recording.
- **Robot Control**: Supports both real and fake hardware execution for the UR10.
- **Vision Integration**: RealSense camera integration with ArUco marker detection for precise tracking.
- **Hand-Eye Calibration**: Uses `easy_handeye2` to perform eye-on-base calibration.
- **Arduino Interface**: Communication with an Arduino to read weight sensors and control LED indicators, with a live potting interface.
- **Robot Sequencer**: Executes time-based or feedback-based trajectories with synchronized LED control.
- **Data Recording**: Integrated MCAP bag recording for all critical topics directly from the GUI.

## Prerequisites

- **ROS 2** (Humble or Iron recommended)
- **Python Dependencies**: `PyQt5`, `matplotlib`, `pyserial`
- **ROS 2 Packages Required**:
  - `ur_robot_driver`
  - `realsense2_camera`
  - `aruco_ros`
  - `easy_handeye2`
  - `ur_msgs`, `ur_dashboard_msgs`

## Installation

1. **Source your ROS 2 environment:**
   ```bash
   source /opt/ros/<ros2-distro>/setup.bash
   ```

2. **Create a workspace and clone the repository:**
   ```bash
   mkdir -p ~/ur_sim_ws/src
   cd ~/ur_sim_ws/src
   git clone <repository-url> HRI_UR10
   ```

3. **Install python dependencies:**
   First, ensure `pip3` is installed:
   ```bash
   sudo apt update
   sudo apt install python3-pip
   ```
   Then install the required Python libraries:
   ```bash
   pip3 install PyQt5 matplotlib pyserial
   ```

4. **Install RealSense SDK and ROS 2 Wrapper:**
   The RealSense SDK (librealsense2) and the ROS 2 wrapper are required for the camera.
   ```bash
   sudo apt update
   sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
   ```
   *(Note: The above apt packages typically handle the base librealsense2 SDK automatically in ROS 2. For custom firmware or deeper tools, refer to the [Intel RealSense SDK GitHub](https://github.com/IntelRealSense/librealsense).)*

5. **Install Required ROS 2 Packages:**
   If using `apt`, install the core drivers and ros2-control (using Humble as an example):
   ```bash
   sudo apt update
   sudo apt install ros-humble-ur-robot-driver \
                    ros-humble-aruco-ros \
                    ros-humble-ur-msgs \
                    ros-humble-ur-dashboard-msgs \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers
   ```
   **Note:** `easy_handeye2` currently requires building from source. Clone it into your workspace:
   ```bash
   cd ~/ur_sim_ws/src
   git clone https://github.com/IFL-CAMP/easy_handeye2.git
   ```

6. **Install ROS dependencies using rosdep:**
   ```bash
   cd ~/ur_sim_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

7. **Build the workspace:**
   ```bash
   cd ~/ur_sim_ws
   colcon build --symlink-install
   ```

8. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## UR10 Robot Calibration

Before launching the robot accurately for the first time, it is highly recommended to extract the factory calibration from your specific UR10 robot. This creates a calibration parameter `.yaml` file used by the ROS driver.

1. Ensure your robot is powered on and connected to the network (Default IP: `192.168.11.100`).
2. Run the calibration extraction node (you only need to do this once):
   ```bash
   ros2 launch ur_calibration calibration_launch.py \
       ur_type:=ur10 \
       robot_ip:=192.168.11.100 \
       target_filename:="${HOME}/my_ur10_calibration.yaml"
   ```
3. This creates `my_ur10_calibration.yaml` in your home directory, which the launch GUI (`launch_gui.py`) and scripts automatically utilize when launching the driver.

## Quick Start (GUI)

The primary way to interact with the entire system is through the unified GUI.

1. Start the control center:
   ```bash
   ros2 run ur10_custom_description launch_gui.py
   ```

2. From the GUI, you can easily control:
   - **UR10 Robot**: Configure the robot's IP and toggle Fake Hardware on/off. Click **Start Robot** to launch the `ur_robot_driver`.
   - **Realsense Camera**: Start the depth-enabled pointcloud camera feed.
   - **Arduino Monitor**: Select your port (`/dev/ttyACM0`) and click **Start Driver** to view live weight status and sync LED commands.
   - **Robot Sequencer**: Input a custom time span (e.g. `1, 2, 1`), select LED logic (RANDOM/RED/BLUE), and execute automated cycles.
   - **Handeye Calibration**: Provide an Eye-on-Base calibration name to quickly publish TF data.
   - **Recording**: Seamlessly start logging core topics to MCAP bagfiles for playback and analysis.

## Manual Launching (Advanced)

If you prefer to bypass the UI, each component can be launched individually:

**UR10 Robot Driver:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.11.100 launch_rviz:=false kinematics_params_file:="${HOME}/my_ur10_calibration.yaml"
```

**RealSense Camera:**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true rgb_camera.profile:=640x480x30 depth_module.depth_profile:=640x480x30
```

**ArUco Marker Node:**
```bash
ros2 run aruco_ros single --ros-args -p marker_id:=582 -p marker_size:=0.15 -p reference_frame:=camera_color_optical_frame -p camera_frame:=camera_color_optical_frame -p marker_frame:=marker_frame -r /camera_info:=/camera/camera/color/camera_info -r /image:=/camera/camera/color/image_raw
```

**Arduino Interface Bridge:**
```bash
ros2 launch ur10_cyclic arduino_interface.launch.py
ros2 run ur10_cyclic live_plotter
```

## Maintenance & Recovery

- **Cleanup Ghost Nodes**: If any lingering processes remain active after an anomalous shutdown, run:
  ```bash
  pkill -f ros; pkill -f gzserver; pkill -f gzclient; pkill -f rviz; ros2 daemon stop && ros2 daemon start
  ```
- **Syncing System Time (Robot)**: Ensure the robot controller clock is synced for bag recording:
  ```bash
  ssh root@192.168.11.100 date
  sudo date -s "<Current Date in UTC>"
  ```
