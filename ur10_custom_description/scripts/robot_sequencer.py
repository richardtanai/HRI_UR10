#!/usr/bin/env python3
import rclpy
import time
import threading
import random
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String, Empty
from sensor_msgs.msg import JointState
from ur_msgs.msg import IOStates

class RobotSequencer(Node):
    def __init__(self):
        super().__init__('robot_sequencer')
        
        # --- Action Client ---
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # UR10 Standard Joint Names
        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # --- Poses (Radians) ---
        # Cycle: Start -> Point A -> Start
        self.poses = {
            # "Start":  [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
            # "Start":  [-1.5672, -1.6651, -1.8352, -3.2321, -1.5942, -0.0002],
            "Start":  [-1.5637, -1.8137, -1.9363, -2.6660, -1.5511, -3.1461],
            # "Point A": [-1.57, -2.00, -2.00, -1.00, 1.57, 0.0]
            "Point A": [-1.5637, -2.0528, -1.9726, -2.6660, -1.5511, -3.1462],
        }
        self.cycle_order = ["Start", "Point A", "Start"]

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            String,
            '/robot_sequence/durations',
            self.listener_callback,
            10
        )
        self.io_sub = self.create_subscription(
            IOStates,
            '/io_and_status_controller/io_states',
            self.io_callback,
            10
        )
        
        self.subscription_fb = self.create_subscription(
            String,
            '/robot_sequence/durations_feedback',
            self.listener_callback_fb,
            10
        )
        # --- Series Subscribers (pre-computed colors, single trajectory) ---
        self.series_sub = self.create_subscription(
            String,
            '/robot_sequence/series',
            self.series_callback,
            10
        )
        self.series_fb_sub = self.create_subscription(
            String,
            '/robot_sequence/series_feedback',
            self.series_callback_fb,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # --- Service ---
        from std_srvs.srv import Trigger
        self.srv_start = self.create_service(Trigger, '/robot_sequence/move_to_start', self.move_to_start_callback)
        
        # --- Status Publisher (for Series Composer in GUI) ---
        self.status_pub = self.create_publisher(String, '/robot_sequence/status', 10)
        self.pub_timing_robot = self.create_publisher(Empty, '/perfect_timing_robot', 10)
        self.pub_timing_human = self.create_publisher(Empty, '/perfect_timing_human', 10)
        
        self.execution_thread = None
        self.stop_flag = False
        self.current_joint_positions = None

        self.get_logger().info("Robot Sequencer Ready. Listening on /robot_sequence/durations")

    def io_callback(self, msg):
        for digital_in in msg.digital_in_states:
            if digital_in.pin == 3 and not digital_in.state:
                if not self.stop_flag:
                    self.get_logger().warn("SOFT ESTOP Detected! Aborting sequence.")
                    self.stop_flag = True

    def joint_state_callback(self, msg):
        # Dynamically map the joint positions to match our joint_names order
        pos_dict = dict(zip(msg.name, msg.position))
        if all(name in pos_dict for name in self.joint_names):
            self.current_joint_positions = [pos_dict[name] for name in self.joint_names]

    def move_to_start_callback(self, request, response):
        self.get_logger().info("Received Request: Move to Start")
        
        # Stop existing sequence
        if self.execution_thread and self.execution_thread.is_alive():
            self.stop_flag = True
            self.execution_thread.join(timeout=1.0)
            
        # Move to Start in new thread (to avoid blocking service)
        threading.Thread(target=self.execute_single_move, args=("Start", 4.0)).start()
        
        response.success = True
        response.message = "Moving to Start Position..."
        return response

    def execute_single_move(self, pose_name, duration):
         target_joints = self.poses[pose_name]
         success = self.send_goal(target_joints, duration)
         if success: self.get_logger().info(f"Reached {pose_name}")
         else: self.get_logger().error(f"Failed to reach {pose_name}")

    def listener_callback(self, msg):
        try:
            # Parse payload: "1, 2, 1 | 50 | RANDOM"
            data_parts = msg.data.split('|')
            durations_str = data_parts[0]
            red_prob = int(data_parts[1].strip()) if len(data_parts) > 1 else 50 # Default 50%
            led_mode = data_parts[2].strip().upper() if len(data_parts) > 2 else "RANDOM"
            
            durations = [float(x.strip()) for x in durations_str.split(',') if x.strip()]
            self.get_logger().info(f"Received Sequence (Time-Based): {durations} with Red Prob: {red_prob}%, LED Mode: {led_mode}")
            
            if self.execution_thread and self.execution_thread.is_alive():
                self.get_logger().warn("Sequence already running. Ignoring new request.")
            else:
                self.stop_flag = False
                self.execution_thread = threading.Thread(target=self.execute_sequence, args=(durations, red_prob, led_mode))
                self.execution_thread.start()
                
        except ValueError:
            self.get_logger().error(f"Invalid format: {msg.data}")

    def listener_callback_fb(self, msg):
        try:
            data_parts = msg.data.split('|')
            durations_str = data_parts[0]
            red_prob = int(data_parts[1].strip()) if len(data_parts) > 1 else 50
            led_mode = data_parts[2].strip().upper() if len(data_parts) > 2 else "RANDOM"
            
            durations = [float(x.strip()) for x in durations_str.split(',') if x.strip()]
            self.get_logger().info(f"Received Sequence (Feedback-Based): {durations} with Red Prob: {red_prob}%, LED Mode: {led_mode}")
            
            if self.execution_thread and self.execution_thread.is_alive():
                self.get_logger().warn("Sequence already running. Ignoring new request.")
            else:
                self.stop_flag = False
                self.execution_thread = threading.Thread(target=self.execute_sequence_feedback, args=(durations, red_prob, led_mode))
                self.execution_thread.start()
                
        except ValueError:
            self.get_logger().error(f"Invalid format: {msg.data}")

    def series_callback(self, msg):
        """Handle series: 'dur1,dur2,...|color1,color2,...'"""
        try:
            data_parts = msg.data.split('|')
            durations = [float(x.strip()) for x in data_parts[0].split(',') if x.strip()]
            colors = [x.strip().lower() for x in data_parts[1].split(',') if x.strip()]
            
            if len(colors) != len(durations):
                self.get_logger().error(f"Series mismatch: {len(durations)} durations vs {len(colors)} colors")
                return
            
            self.get_logger().info(f"Received Series (Time-Based): {len(durations)} cycles")
            
            if self.execution_thread and self.execution_thread.is_alive():
                self.get_logger().warn("Sequence already running. Ignoring.")
            else:
                self.stop_flag = False
                self.execution_thread = threading.Thread(
                    target=self.execute_series, args=(durations, colors))
                self.execution_thread.start()
        except Exception as e:
            self.get_logger().error(f"Series parse error: {e}")

    def series_callback_fb(self, msg):
        """Handle series feedback: 'dur1,dur2,...|color1,color2,...'"""
        try:
            data_parts = msg.data.split('|')
            durations = [float(x.strip()) for x in data_parts[0].split(',') if x.strip()]
            colors = [x.strip().lower() for x in data_parts[1].split(',') if x.strip()]
            
            if len(colors) != len(durations):
                self.get_logger().error(f"Series mismatch: {len(durations)} durations vs {len(colors)} colors")
                return
            
            self.get_logger().info(f"Received Series (Feedback-Based): {len(durations)} cycles")
            
            if self.execution_thread and self.execution_thread.is_alive():
                self.get_logger().warn("Sequence already running. Ignoring.")
            else:
                self.stop_flag = False
                self.execution_thread = threading.Thread(
                    target=self.execute_series_feedback, args=(durations, colors))
                self.execution_thread.start()
        except Exception as e:
            self.get_logger().error(f"Series parse error: {e}")

    def execute_sequence(self, durations, red_prob=50, led_mode="RANDOM"):
        self.status_pub.publish(String(data="RUNNING"))
        # build the trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        current_time = 0.0
        points = []
        
        # We need an LED publisher here if one wasn't made in init
        if not hasattr(self, 'led_pub'):
            self.led_pub = self.create_publisher(String, '/arduino/led_color_cmd', 10)
        
        led_colors = [] # Store which color corresponds to which part of the trajectory
        
        for cycle_idx, duration in enumerate(durations):
            self.get_logger().info(f"--- Cycle {cycle_idx+1}: Duration {duration}s per move ---")
            
            # Determine color for this cycle
            if led_mode == "RANDOM":
                rand_val = random.randint(1, 100)
                color = "red" if rand_val <= red_prob else "blue"
                self.get_logger().info(f"Cycle {cycle_idx+1} LED Color selected as: {color} (Rolled {rand_val} vs {red_prob}%)")
            elif led_mode == "RED":
                color = "red"
            elif led_mode == "BLUE":
                color = "blue"
            else:
                color = "off"
            
            # Cycle: Point A -> Start
            for pose_name in ["Point A", "Start"]:
                if self.stop_flag: 
                    self.get_logger().info("Sequence Stopped during construction.")
                    return

                current_time += duration
                
                point = JointTrajectoryPoint()
                point.positions = [float(x) for x in self.poses[pose_name]]
                # Setting 0.0 velocity and acceleration forces the driver to smoothly 
                # accelerate out of and decelerate into the waypoints, stopping jerk.
                point.velocities = [0.0] * len(self.joint_names)
                point.accelerations = [0.0] * len(self.joint_names)
                
                sec = int(current_time)
                nanosec = int((current_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                
                points.append(point)
                if pose_name == "Point A":
                    led_colors.append(color)
                else:
                    led_colors.append("off") # Turn off LED on return trip to mimic Arduino pulsing

        goal_msg.trajectory.points = points
        
        self.get_logger().info(f"Sending trajectory with {len(points)} points. Total time: {current_time}s")
        
        # Send the full trajectory
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            return

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for acceptance
        while not send_goal_future.done():
            if self.stop_flag: return
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        get_result_future = goal_handle.get_result_async()
        
        # Start time for tracking which waypoint we're on
        start_time = time.time()
        last_color_idx = -1
        next_waypoint_to_pass = 0
        
        # Wait for execution
        while not get_result_future.done():
            if self.stop_flag:
                self.get_logger().info("Cancelling goal...")
                goal_handle.cancel_goal_async()
                # Turn off LED on stop
                stop_msg = String()
                stop_msg.data = "off"
                self.led_pub.publish(stop_msg)
                return
                
            # Time elapsed since trajectory started
            elapsed = time.time() - start_time
            
            while next_waypoint_to_pass < len(points):
                p = points[next_waypoint_to_pass]
                wp_time = p.time_from_start.sec + (p.time_from_start.nanosec * 1e-9)
                if elapsed >= wp_time:
                    if next_waypoint_to_pass % 2 == 0:
                        self.pub_timing_robot.publish(Empty())
                    else:
                        self.pub_timing_human.publish(Empty())
                    next_waypoint_to_pass += 1
                else:
                    break
                    
            # Find which waypoint we are currently executing
            current_target_idx = 0
            for i, p in enumerate(points):
                wp_time = p.time_from_start.sec + (p.time_from_start.nanosec * 1e-9)
                if elapsed < wp_time:
                    current_target_idx = i
                    break
            else:
                current_target_idx = len(points) - 1 # Last point if we exceeded time
            
            # Publish color if it changed
            if current_target_idx != last_color_idx:
                color_msg = String()
                color_msg.data = led_colors[current_target_idx]
                self.led_pub.publish(color_msg)
                last_color_idx = current_target_idx
                
            time.sleep(0.05)
            
        result = get_result_future.result().result
        
        # Turn off LED when done
        off_msg = String()
        off_msg.data = "off"
        self.led_pub.publish(off_msg)
        
        self.status_pub.publish(String(data="COMPLETED"))
        self.get_logger().info("Sequence Completed.")

    def execute_sequence_feedback(self, durations, red_prob=50, led_mode="RANDOM"):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        current_time = 0.0
        points = []
        
        if not hasattr(self, 'led_pub'):
            self.led_pub = self.create_publisher(String, '/arduino/led_color_cmd', 10)
        
        # Colors planned per cycle (for the forward stroke)
        cycle_colors = []
        for cycle_idx, duration in enumerate(durations):
            if led_mode == "RANDOM":
                rand_val = random.randint(1, 100)
                color = "red" if rand_val <= red_prob else "blue"
            elif led_mode == "RED": color = "red"
            elif led_mode == "BLUE": color = "blue"
            else: color = "off"
            
            cycle_colors.append(color)
            for pose_name in ["Point A", "Start"]:
                current_time += duration
                point = JointTrajectoryPoint()
                point.positions = [float(x) for x in self.poses[pose_name]]
                # Smooth the stroke by explicitly decelerating to 0 at the end of the swing
                point.velocities = [0.0] * len(self.joint_names)
                point.accelerations = [0.0] * len(self.joint_names)
                sec = int(current_time)
                nanosec = int((current_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                points.append(point)

        goal_msg.trajectory.points = points
        self.get_logger().info(f"Sending trajectory Feedback-Based. Total time: {current_time}s")
        
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            return

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            if self.stop_flag: return
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        get_result_future = goal_handle.get_result_async()
        
        # Real-time feedback loop
        last_led_state = ""
        shoulder_idx = self.joint_names.index("shoulder_lift_joint")
        
        # Cycle tracking
        cycle_idx = 0
        point_a_shoulder_lift = self.poses["Point A"][shoulder_idx] # -2.05
        start_shoulder_lift = self.poses["Start"][shoulder_idx]     # -1.81
        
        # Hysteresis Thresholds (Position in space)
        # Sequence: Start = -1.81, Point A = -2.05
        # The threshold must have a gap to prevent flickering. 
        # turn_on must be LESS THAN turn_off (farther into the stroke).
        turn_on_threshold = -1.86  # Turn ON when moving past -1.86 towards -2.05
        turn_off_threshold = -1.83 # Turn OFF when moving past -1.83 back towards -1.81
        
        # State tracking
        is_extended_zone = False 
        
        while not get_result_future.done():
            if self.stop_flag:
                goal_handle.cancel_goal_async()
                self.led_pub.publish(String(data="off"))
                return
                
            if self.current_joint_positions:
                curr_shoulder_lift = self.current_joint_positions[shoulder_idx]
                
                # Hysteresis Logic
                if curr_shoulder_lift < turn_on_threshold and not is_extended_zone:
                    # Crossed the threshold extending outwards
                    is_extended_zone = True
                    self.pub_timing_robot.publish(Empty())
                    # Grab color for this cycle iteration safely
                    desired_state = cycle_colors[cycle_idx] if cycle_idx < len(cycle_colors) else cycle_colors[-1]
                    
                    if desired_state != last_led_state and desired_state != "":
                        self.led_pub.publish(String(data=desired_state))
                        last_led_state = desired_state
                        
                elif curr_shoulder_lift > turn_off_threshold and is_extended_zone:
                    # Crossed the threshold retracting inwards
                    is_extended_zone = False
                    self.pub_timing_human.publish(Empty())
                    desired_state = "off"
                    
                    if desired_state != last_led_state:
                        self.led_pub.publish(String(data=desired_state))
                        last_led_state = desired_state
                        
                    # Successfully returned to start, advance the color sequence!
                    # Only advance if we haven't exhausted the planned colors
                    if cycle_idx < len(cycle_colors) - 1:
                        cycle_idx += 1
                        self.get_logger().info(f"Feedback Sequencer advanced to Cycle {cycle_idx+1}")
                
            time.sleep(0.01) # Fast polling for feedback
            
        self.led_pub.publish(String(data="off"))
        self.status_pub.publish(String(data="COMPLETED"))
        self.get_logger().info("Feedback Sequence Completed.")

    def execute_series(self, durations, colors):
        """Execute a series as one continuous trajectory with pre-resolved per-cycle colors."""
        self.status_pub.publish(String(data="RUNNING"))
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        current_time = 0.0
        points = []
        
        if not hasattr(self, 'led_pub'):
            self.led_pub = self.create_publisher(String, '/arduino/led_color_cmd', 10)
        
        led_colors = []
        
        for cycle_idx, duration in enumerate(durations):
            color = colors[cycle_idx]
            self.get_logger().info(f"--- Series Cycle {cycle_idx+1}: Duration {duration}s, Color: {color} ---")
            
            for pose_name in ["Point A", "Start"]:
                if self.stop_flag:
                    self.get_logger().info("Series stopped during construction.")
                    return

                current_time += duration
                
                point = JointTrajectoryPoint()
                point.positions = [float(x) for x in self.poses[pose_name]]
                point.velocities = [0.0] * len(self.joint_names)
                point.accelerations = [0.0] * len(self.joint_names)
                
                sec = int(current_time)
                nanosec = int((current_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                
                points.append(point)
                if pose_name == "Point A":
                    led_colors.append(color)
                else:
                    led_colors.append("off")

        goal_msg.trajectory.points = points
        
        self.get_logger().info(f"Sending series trajectory with {len(points)} points. Total time: {current_time}s")
        
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            return

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        while not send_goal_future.done():
            if self.stop_flag: return
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        get_result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        last_color_idx = -1
        next_waypoint_to_pass = 0
        
        while not get_result_future.done():
            if self.stop_flag:
                self.get_logger().info("Cancelling series goal...")
                goal_handle.cancel_goal_async()
                stop_msg = String()
                stop_msg.data = "off"
                self.led_pub.publish(stop_msg)
                return
                
            elapsed = time.time() - start_time
            
            while next_waypoint_to_pass < len(points):
                p = points[next_waypoint_to_pass]
                wp_time = p.time_from_start.sec + (p.time_from_start.nanosec * 1e-9)
                if elapsed >= wp_time:
                    if next_waypoint_to_pass % 2 == 0:
                        self.pub_timing_robot.publish(Empty())
                    else:
                        self.pub_timing_human.publish(Empty())
                    next_waypoint_to_pass += 1
                else:
                    break
                    
            current_target_idx = 0
            for i, p in enumerate(points):
                wp_time = p.time_from_start.sec + (p.time_from_start.nanosec * 1e-9)
                if elapsed < wp_time:
                    current_target_idx = i
                    break
            else:
                current_target_idx = len(points) - 1
            
            if current_target_idx != last_color_idx:
                color_msg = String()
                color_msg.data = led_colors[current_target_idx]
                self.led_pub.publish(color_msg)
                last_color_idx = current_target_idx
                
            time.sleep(0.05)
            
        result = get_result_future.result().result
        
        off_msg = String()
        off_msg.data = "off"
        self.led_pub.publish(off_msg)
        
        self.status_pub.publish(String(data="COMPLETED"))
        self.get_logger().info("Series Completed.")

    def execute_series_feedback(self, durations, colors):
        """Execute a series as one continuous trajectory with feedback-based LED and pre-resolved colors."""
        self.status_pub.publish(String(data="RUNNING"))
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        current_time = 0.0
        points = []
        
        if not hasattr(self, 'led_pub'):
            self.led_pub = self.create_publisher(String, '/arduino/led_color_cmd', 10)
        
        cycle_colors = list(colors)  # Pre-resolved, use directly
        for cycle_idx, duration in enumerate(durations):
            for pose_name in ["Point A", "Start"]:
                current_time += duration
                point = JointTrajectoryPoint()
                point.positions = [float(x) for x in self.poses[pose_name]]
                point.velocities = [0.0] * len(self.joint_names)
                point.accelerations = [0.0] * len(self.joint_names)
                sec = int(current_time)
                nanosec = int((current_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                points.append(point)

        goal_msg.trajectory.points = points
        self.get_logger().info(f"Sending series trajectory (Feedback). Total time: {current_time}s")
        
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            return

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            if self.stop_flag: return
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        get_result_future = goal_handle.get_result_async()
        
        # Real-time feedback loop
        last_led_state = ""
        shoulder_idx = self.joint_names.index("shoulder_lift_joint")
        
        cycle_idx = 0
        turn_on_threshold = -1.86
        turn_off_threshold = -1.83
        is_extended_zone = False 
        
        while not get_result_future.done():
            if self.stop_flag:
                goal_handle.cancel_goal_async()
                self.led_pub.publish(String(data="off"))
                return
                
            if self.current_joint_positions:
                curr_shoulder_lift = self.current_joint_positions[shoulder_idx]
                
                if curr_shoulder_lift < turn_on_threshold and not is_extended_zone:
                    is_extended_zone = True
                    self.pub_timing_robot.publish(Empty())
                    desired_state = cycle_colors[cycle_idx] if cycle_idx < len(cycle_colors) else cycle_colors[-1]
                    
                    if desired_state != last_led_state and desired_state != "":
                        self.led_pub.publish(String(data=desired_state))
                        last_led_state = desired_state
                        
                elif curr_shoulder_lift > turn_off_threshold and is_extended_zone:
                    is_extended_zone = False
                    self.pub_timing_human.publish(Empty())
                    desired_state = "off"
                    
                    if desired_state != last_led_state:
                        self.led_pub.publish(String(data=desired_state))
                        last_led_state = desired_state
                        
                    if cycle_idx < len(cycle_colors) - 1:
                        cycle_idx += 1
                        self.get_logger().info(f"Series Feedback advanced to Cycle {cycle_idx+1}")
                
            time.sleep(0.01)
            
        self.led_pub.publish(String(data="off"))
        self.status_pub.publish(String(data="COMPLETED"))
        self.get_logger().info("Series (Feedback) Completed.")

    def send_goal(self, joint_angles, duration_sec):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Action server not available!")
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in joint_angles]
        
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        goal_msg.trajectory.points = [point]
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        while not send_goal_future.done():
            if self.stop_flag: return False
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        get_result_future = goal_handle.get_result_async()
        
        # Wait for execution
        while not get_result_future.done():
            if self.stop_flag:
                self.get_logger().info("Cancelling single move goal...")
                goal_handle.cancel_goal_async()
                return False
            time.sleep(0.1)
            
        result = get_result_future.result().result
        return True

def main(args=None):
    rclpy.init(args=args)
    node = RobotSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
