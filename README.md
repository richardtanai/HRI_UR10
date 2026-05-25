# HRI_UR10: Human-Robot Interaction with UR10

This repository contains an integrated ROS 2 system for controlling a UR10 robot, an Intel RealSense camera, and an Arduino-based sensor/LED interface. It features a comprehensive PyQt5-based GUI for launching and monitoring the entire system, performing hand-eye calibration, and executing automated robot sequences.

---

## Table of Contents
1. [Features](#features)
2. [Prerequisites & System Requirements](#prerequisites--system-requirements)
3. [Network Settings & Firewall Configuration](#1-network-settings-and-firewall-configuration)
4. [Serial Communication Setup](#2-serial-communication-setup)
5. [Workspace Installation & Sourcing](#3-workspace-installation--sourcing)
6. [Hardware & Microcontroller Setup](#4-hardware--microcontroller-setup)
7. [Robot Calibration Extraction](#5-robot-calibration-extraction)
8. [Quick Start (Unified Launch GUI)](#6-quick-start-unified-launch-gui)
9. [Manual Launching & Advanced Commands](#7-manual-launching--advanced-commands)
10. [Maintenance, Troubleshooting & Recovery](#8-maintenance-troubleshooting--recovery)

---

## Features
- **Integrated Launch GUI**: A central control center (`launch_gui.py`) to start/stop the UR10 driver, RealSense camera, Arduino bridge, Hand-eye calibration, and recording.
- **Robot Control**: Supports both real and fake hardware execution for the UR10.
- **Vision Integration**: RealSense camera integration with ArUco marker detection for precise tracking.
- **Hand-Eye Calibration**: Uses `easy_handeye2` to perform eye-on-base calibration.
- **Arduino Interface**: Communication with an Arduino to read weight sensors and control LED indicators, with a live plotting interface.
- **Robot Sequencer**: Executes time-based or feedback-based trajectories with synchronized LED control.
- **Data Recording**: Integrated MCAP bag recording for all critical topics directly from the GUI.

---

## Prerequisites & System Requirements
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) or similar Linux system
- **ROS 2**: Humble (recommended) or Iron
- **External Python System Packages**: 
  - `ultralytics` (Must be installed via `pip3` since it's not a standard ROS package)
  - Python bindings/libraries: `PyQt5`, `matplotlib`, `pyserial`

---

## 1. Network Settings and Firewall Configuration
To establish communication with the UR10 robot controller, ensure your host machine is correctly configured.

### Network Configuration
- Set your PC's Ethernet adapter to a static IP address within the same subnet as the UR10 controller:
  - **Example Robot IP**: `192.168.11.100`
  - **Example PC IP**: `192.168.11.71`
  - **Subnet Mask**: `255.255.255.0`

### Firewall Configuration
ROS 2 and the UR driver require specific network ports to be open for communication.
- Ensure that your firewall is either disabled for the local subnet or configured to allow ROS 2 DDS traffic (typically UDP ports in the 7400+ range) and UR TCP/IP ports (e.g., 50001, 50002, 50003, 30001-30004).
- To disable the UFW firewall temporarily for testing:
  ```bash
  sudo ufw disable
  ```

---

## 2. Serial Communication Setup
For devices like the Arduino or serial grippers to communicate properly via USB/Serial, your user must be added to the `dialout` group.
```bash
sudo usermod -aG dialout $USER
```
> [!NOTE]
> You must log out and log back in (or restart your computer) for this group change to take effect.

---

## 3. Workspace Installation & Sourcing

### A. Installing External System & Python Dependencies
First, ensure standard tools and dependencies are installed:
```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep build-essential git curl libfuse2
```

Install python-specific requirements in the python system:
```bash
pip3 install PyQt5 matplotlib pyserial ultralytics
```
> [!IMPORTANT]
> The `ultralytics` (YOLO) library is required by the `human_safety_node` and cannot be installed via `rosdep install` because no standard Ubuntu rosdep key exists. You must install it manually via the pip command above.

### B. Installing Intel RealSense SDK & ROS 2 Wrapper
```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description
```

### C. Creating the Workspace & Cloning
Create a Colcon workspace and clone this repository:
```bash
mkdir -p ~/hri_ws/src
cd ~/hri_ws/src
git clone <repository-url> HRI_UR10
```

### D. Running rosdep and Building the Workspace
Ensure you have initialized and updated `rosdep` before installing:
```bash
sudo rosdep init
rosdep update
```

Install all ROS package dependencies automatically using `rosdep`:
```bash
cd ~/hri_ws
rosdep install --from-paths src --ignore-src -r -y
```

Now build the workspace. (Note: `easy_handeye2` and `aruco_ros` are built from source within this repository).
```bash
colcon build --symlink-install
```

### E. Sourcing the Workspace
Source the workspace after a successful build:
```bash
source install/setup.bash
```

---

## 4. Hardware & Microcontroller Setup
If your setup utilizes an Arduino (e.g., for cyclic signaling, hand-tracking bridging, or external sensor data), flash the corresponding sketch:

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) or use `arduino-cli`.
2. Install any required libraries via the Arduino Library Manager.
3. Open the provided `.ino` sketch file located at `src/HRI_UR10/arduino/ros_interface/ros_interface.ino`.
4. Select the correct Board and Port in the Arduino IDE (`Tools > Board`, `Tools > Port`).
5. Click **Upload** to compile and flash the sketch to the Arduino.

---

## 5. Robot Calibration Extraction
Before launching the robot accurately for the first time, it is highly recommended to extract the factory calibration from your specific UR10 robot. This creates a calibration parameter `.yaml` file used by the ROS driver.

1. Ensure your robot is powered on and connected to the network (Default IP: `192.168.11.100`).
2. Run the calibration extraction node (you only need to do this once):
   ```bash
   ros2 launch ur_calibration calibration_launch.py \
       ur_type:=ur10 \
       robot_ip:=192.168.11.100 \
       target_filename:="${HOME}/my_ur10_calibration.yaml"
   ```
3. This creates `my_ur10_calibration.yaml` in your home directory, which the launch GUI and scripts automatically utilize when launching the driver.

---

## 6. Quick Start (Unified Launch GUI)
The primary way to interact with the entire system is through the unified PyQt5 GUI.

1. Source your workspace:
   ```bash
   source ~/hri_ws/install/setup.bash
   ```
2. Start the control center GUI:
   ```bash
   ros2 run ur10_custom_description launch_gui.py
   ```
3. From the GUI, you can easily control:
   - **UR10 Robot**: Configure the robot's IP and toggle Fake Hardware on/off. Click **Start Robot** to launch the `ur_robot_driver`.
   - **Realsense Camera**: Start the depth-enabled pointcloud camera feed.
   - **Arduino Monitor**: Select your port (e.g., `/dev/ttyACM0`) and click **Start Driver** to view live weight status and sync LED commands.
   - **Robot Sequencer**: Input a custom time span (e.g. `1, 2, 1`), select LED logic (RANDOM/RED/BLUE), and execute automated cycles.
   - **Handeye Calibration**: Provide an Eye-on-Base calibration name to quickly publish TF data.
   - **Recording**: Seamlessly start logging core topics to MCAP bagfiles for playback and analysis.

Alternatively, to run the helper calibration GUI:
```bash
ros2 run ur10_calibration calibration_gui
# Or via its launch file:
ros2 launch ur10_calibration ur10_calibration.launch.py
```

---

## 7. Manual Launching & Advanced Commands
If you prefer to bypass the UI, each component can be launched manually:

### A. UR10 Robot Driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10 \
    robot_ip:=192.168.11.100 \
    launch_rviz:=false \
    kinematics_params_file:="${HOME}/my_ur10_calibration.yaml"
```

### B. Intel RealSense Camera
- **With Depth Cloud**:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
      align_depth.enable:=true \
      pointcloud.enable:=true \
      rgb_camera.profile:=1280x720x30 \
      depth_module.depth_profile:=1280x720x30
  ```
- **Reduced Resolution**:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
      align_depth.enable:=true \
      pointcloud.enable:=true \
      rgb_camera.profile:=640x480x30 \
      depth_module.depth_profile:=640x480x30
  ```

### C. ArUco Marker Detection Node
```bash
ros2 run aruco_ros single --ros-args \
    -p marker_id:=582 \
    -p marker_size:=0.15 \
    -p reference_frame:=camera_color_optical_frame \
    -p camera_frame:=camera_color_optical_frame \
    -p marker_frame:=marker_frame \
    -r /camera_info:=/camera/camera/color/camera_info \
    -r /image:=/camera/camera/color/image_raw
```

### D. Hand-Eye Calibration Nodes
- **For `camera_color_optical_frame`**:
  ```bash
  ros2 launch easy_handeye2 calibrate.launch.py \
      name:=ur10_realsense_eob2 \
      calibration_type:=eye_on_base \
      robot_base_frame:=base_link \
      robot_effector_frame:=tool0 \
      tracking_base_frame:=camera_color_optical_frame \
      tracking_marker_frame:=marker_frame \
      freehand_robot_movement:=true
  ```
- **For `camera_link`**:
  ```bash
  ros2 launch easy_handeye2 calibrate.launch.py \
      name:=ur10_realsense_eob6 \
      calibration_type:=eye_on_base \
      robot_base_frame:=base_link \
      robot_effector_frame:=tool0 \
      tracking_base_frame:=camera_link \
      tracking_marker_frame:=marker_frame \
      freehand_robot_movement:=true
  ```
- **Publish Calibration Result**:
  ```bash
  ros2 launch easy_handeye2 publish.launch.py name:=ur10_realsense_eob
  ```
- **Evaluate & Verify Calibration Result (Check Error)**:
  ```bash
  ros2 launch easy_handeye2 evaluate.launch.py name:=ur10_realsense_eob
  ```

### E. Arduino & Sensor Bridge
- **Upload** `ros_interface.ino` using the Arduino IDE.
- **Run the Arduino Bridge**:
  ```bash
  ros2 run ur10_cyclic arduino_bridge
  # Or launch both interface & bridge:
  ros2 launch ur10_cyclic arduino_interface.launch.py
  ```
- **Run the Live Plotter**:
  ```bash
  ros2 run ur10_cyclic live_plotter
  ```
- **Testing LED & Sensor Commands**:
  - Start light sequence:
    ```bash
    ros2 topic pub /arduino/command std_msgs/String "data: '1'"
    ```
  - Stop light sequence:
    ```bash
    ros2 topic pub /arduino/command std_msgs/String "data: 's'"
    ```
  - Listen to weight readings:
    ```bash
    ros2 topic echo /arduino/weight
    ```

### F. Robot Aruco Sequencer
- **Run the Sequence Node**:
  ```bash
  ros2 run ur10_cyclic aruco_sequence
  ```
- **Control via GUI**: Run the controller in a separate terminal:
  ```bash
  ros2 run ur10_cyclic sequence_controller
  ```
- **Control via Command Line Services**:
  - Trigger next pose:
    ```bash
    ros2 service call /aruco_sequence/next_pose std_srvs/srv/Trigger
    ```
  - Reset sequence:
    ```bash
    ros2 service call /aruco_sequence/reset std_srvs/srv/Trigger
    ```

---

## 8. Maintenance, Troubleshooting & Recovery

- **Cleanup Lingering Nodes**: If any processes remain active after an anomalous shutdown, clean them using:
  ```bash
  pkill -f ros; pkill -f gzserver; pkill -f gzclient; pkill -f rviz; pkill -f move_group; pkill -f robot_state_publisher; pkill -f controller_manager; pkill -f calibration_gui
  ```
- **Restart ROS 2 Daemon**:
  ```bash
  ros2 daemon stop && ros2 daemon start
  ```
- **Syncing Robot System Time**: Ensure the robot controller clock is synced to the PC clock for recording accuracy:
  ```bash
  ssh root@192.168.11.100 date
  sudo date -s "<Current Date/Time in UTC>"
  ```
