# Custom UR10 with Human Fist - Usage Guide

## Package Overview
This package provides a custom UR10 robot description with a static human fist attachment at tool0.

## Features
- **Human Fist Attachment**: 12cm x 8cm x 10cm, 0.5kg static fist model
- **Fake Hardware Support**: Test algorithms without physical robot
- **RViz Visualization**: Interactive robot model viewing
- **Controller Support**: Joint trajectory and position controllers

## Quick Start

### 1. Build the Package
```bash
cd ~/ur_sim_ws
colcon build --packages-select ur10_custom_description
source install/setup.bash
```

### 2. Visualize in RViz
```bash
ros2 launch ur10_custom_description view_robot.launch.py
```
This opens RViz with the robot model and a GUI to move joints.

### 3. Launch with Fake Hardware
```bash
ros2 launch ur10_custom_description ur10_custom.launch.py use_fake_hardware:=true
```
This starts the robot with fake hardware controllers for algorithm testing.

## Launch Files

### `view_robot.launch.py`
Visualize the robot in RViz with joint state publisher GUI.
- **Usage**: `ros2 launch ur10_custom_description view_robot.launch.py`
- **Purpose**: Design verification, visualization

### `ur10_custom.launch.py`
Main launch file with controller support.
- **Arguments**:
  - `use_fake_hardware:=true/false` - Enable fake hardware mode (default: false)
  - `robot_ip:=<IP>` - Robot IP address for real hardware (default: 192.168.1.102)
  - `ur_type:=ur10` - UR robot type
- **Usage**: 
  ```bash
  # Fake hardware mode
  ros2 launch ur10_custom_description ur10_custom.launch.py use_fake_hardware:=true
  
  # Real hardware mode
  ros2 launch ur10_custom.description ur10_custom.launch.py robot_ip:=192.168.1.102
  ```

## Robot Specifications

### Human Fist Attachment
- **Location**: Mounted at `tool0` frame
- **Dimensions**: 
  - Length: 12cm (along Z-axis from tool0)
  - Width: 8cm
  - Height: 10cm
- **Mass**: 0.5kg
- **Color**: Grey (RGB: 0.5, 0.5, 0.5)
- **Frames**:
  - `fist_link` - Main fist body
  - `fist_tip_link` - Tip of the fist (12cm from tool0)

### Customization
To modify the fist dimensions, edit `/urdf/human_fist.xacro`:
```xml
<xacro:property name="fist_length" value="0.12" />  <!-- meters -->
<xacro:property name="fist_width" value="0.08" />
<xacro:property name="fist_height" value="0.10" />
<xacro:property name="fist_mass" value="0.5" />     <!-- kg -->
```

## Testing Algorithms with Fake Hardware

The fake hardware mode allows you to test motion planning and control algorithms without the physical robot:

```bash
# Terminal 1: Start robot with fake hardware
ros2 launch ur10_custom_description ur10_custom.launch.py use_fake_hardware:=true

# Terminal 2: Send commands to the robot
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, 
                    wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [
        { positions: [0, -1.57, 0, -1.57, 0, 0], time_from_start: {sec: 2} }
      ]
    }
  }"
```

## TF Frames
The robot provides the following key frames:
- `world` - Base frame
- `base_link` - Robot base
- `tool0` - Standard UR tool flange
- `fist_link` - Human fist attachment
- `fist_tip_link` - Tip of the fist

## Next Steps
- Add custom mesh files to `/meshes/` directory for more realistic fist model
- Integrate with MoveIt for motion planning
- Add sensors (camera, force/torque) to the fist if needed
