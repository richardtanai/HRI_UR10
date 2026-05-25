# Installation and Setup Guide

This document outlines the necessary steps to set up the environment, configure networking, and run the UR10 robotic system and its associated interfaces.

## 1. Network Settings and Firewall Configuration

To establish communication with the UR10 robot controller, ensure your host machine is correctly configured.

### Network Configuration
- Set your PC's Ethernet adapter to a static IP address within the same subnet as the UR10 controller.
  - **Example Robot IP:** `192.168.11.100`
  - **Example PC IP:** `192.168.11.71`
  - **Subnet Mask:** `255.255.255.0`

### Firewall Configuration
ROS 2 and the UR driver require specific network ports to be open for communication.
- Ensure that your firewall is either disabled for the local subnet or configured to allow ROS 2 DDS traffic (typically UDP ports in the 7400+ range) and UR TCP/IP ports (e.g., 50001, 50002, 50003, 30001-30004).
- To disable the UFW firewall temporarily for testing:
  ```bash
  sudo ufw disable
  ```

## 2. System Dependencies and ROS 2 Packages

Install the necessary system utilities and ROS 2 apt dependencies.

```bash
sudo apt update
sudo apt upgrade -y

# Install standard dependencies
sudo apt install -y python3-pip python3-rosdep2 build-essential git curl

# Install ROS 2 Humble dependencies for UR and visualization
sudo apt install -y \
    ros-humble-ur \
    ros-humble-ur-robot-driver \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv
```

*(Ensure you have initialized and updated rosdep before building the workspace)*
```bash
sudo rosdep init
rosdep update
```

## 3. Serial Communication Setup

For devices like the Arduino or serial grippers to communicate properly via USB/Serial, your user must be added to the `dialout` group.

```bash
sudo usermod -aG dialout $USER
```
*Note: You must log out and log back in (or restart your computer) for this group change to take effect.*

## 4. Building the Workspace

The workspace contains customized packages such as `easy_handeye` and `aruco_ros` that require specific build flags to override existing system installations or to allow custom configurations.

Navigate to your workspace root (e.g., `~/ur_sim_ws`) and build using `colcon`:

```bash
cd ~/ur_sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --allow-overrides easy_handeye aruco_ros
```

Source the workspace after a successful build:
```bash
source install/setup.bash
```

## 5. Arduino Sketch Installation

If your setup utilizes an Arduino (e.g., for cyclic signaling, hand-tracking bridging, or external sensor data), you need to flash the corresponding sketch to the microcontroller.

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`.
2. Install any required libraries via the Arduino Library Manager.
3. Open the provided `.ino` sketch file located in your project directory.
4. Select the correct Board and Port in the Arduino IDE (`Tools > Board`, `Tools > Port`).
5. Click **Upload** to compile and flash the sketch to the Arduino.

## 6. Running the System

After building and sourcing your workspace, you can launch the various system components.

### Launching the Main System / Calibration Nodes
Use the launch files to start the necessary drivers and configurations.

```bash
# Example: Launching the main bringup or a specific launch file
ros2 launch <package_name> <launch_file>.launch.py
```

### Running the Calibration GUI
To run the specialized GUI for hand-eye calibration:

```bash
ros2 run ur10_calibration calibration_gui
```

Alternatively, if you want to use the launch file for calibration:
```bash
ros2 launch ur10_calibration ur10_calibration.launch.py
```
