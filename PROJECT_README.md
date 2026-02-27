# launch the robot

ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur10 \
    robot_ip:=192.168.11.100 \
    launch_rviz:=false \
    kinematics_params_file:="${HOME}/my_ur10_calibration.yaml"

# launch the camera

ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30 align_depth.enable:=false


# launh the camera with depth cloud

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true rgb_camera.profile:=1280x720x30 depth_module.depth_profile:=1280x720x30

# reduced

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true rgb_camera.profile:=640x480x30 depth_module.depth_profile:=640x480x30

# launch the aruco marker node

ros2 run aruco_ros single --ros-args \
-p marker_id:=582 \
-p marker_size:=0.15 \
-p reference_frame:=camera_color_optical_frame \
-p camera_frame:=camera_color_optical_frame \
-p marker_frame:=marker_frame \
-r /camera_info:=/camera/camera/color/camera_info \
-r /image:=/camera/camera/color/image_raw

# launch the easy_handeye2 node

ros2 launch easy_handeye2 calibrate.launch.py \
    name:=ur10_realsense_eob2 \
    calibration_type:=eye_on_base \
    robot_base_frame:=base_link \
    robot_effector_frame:=tool0 \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=marker_frame \
    freehand_robot_movement:=true

# launch the correct handeye2 node for camera_link

ros2 launch easy_handeye2 calibrate.launch.py \
    name:=ur10_realsense_eob6 \
    calibration_type:=eye_on_base \
    robot_base_frame:=base_link \
    robot_effector_frame:=tool0 \
    tracking_base_frame:=camera_link \
    tracking_marker_frame:=marker_frame \
    freehand_robot_movement:=true


# To publish the result:
ros2 launch easy_handeye2 publish.launch.py name:=ur10_realsense_eob

# To verify the result (check error):
ros2 launch easy_handeye2 evaluate.launch.py name:=ur10_realsense_eob


# sync the time

ssh root@192.168.11.100 date

sudo date -s "Fri Jan 30 17:42:00 UTC 2026"



# Arduino

Verification Plan
Automated Tests
N/A (Hardware dependent)
Manual Verification
Flash Arduino: Upload ros_interface.ino.
Run Node: ros2 run ur10_cyclic arduino_bridge
Test Commands:
ros2 topic pub /arduino/command std_msgs/String "data: '1'" -> Verify lights start.
ros2 topic pub /arduino/command std_msgs/String "data: 's'" -> Verify lights stop.
Verify Data:
ros2 topic echo /arduino/weight -> Verify streaming numbers.


Run Arduino Interface: ros2 launch ur10_cyclic arduino_interface.launch.py
Run Plotter: ros2 run ur10_cyclic live_plotter


# Run the Arduino visualizer

Run Arduino Interface: ros2 launch ur10_cyclic arduino_interface.launch.py
Run Plotter: ros2 run ur10_cyclic live_plotter


## plot the robot and the keypoints


# robot aruco sequence based on pre determined positions

Run the Sequence Node:

bash
ros2 run ur10_cyclic aruco_sequence
Control via GUI: Run the controller in a separate terminal. It provides "Next Pose" and "Reset" buttons.

bash
ros2 run ur10_cyclic sequence_controller
Control via Command Line (Alternative):

bash
ros2 service call /aruco_sequence/next_pose std_srvs/srv/Trigger
ros2 service call /aruco_sequence/reset std_srvs/srv/Trigger



kill all processes

pkill -f ros; pkill -f gzserver; pkill -f gzclient; pkill -f rviz; pkill -f move_group; pkill -f robot_state_publisher; pkill -f controller_manager; pkill -f calibration_gui

ros2 daemon stop && ros2 daemon start


The first light sequence has problem it flashes then turns off