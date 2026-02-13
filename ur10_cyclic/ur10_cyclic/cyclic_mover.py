import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Imports for the GUI Slider
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

class CyclicMover(Node):
    def __init__(self):
        super().__init__('cyclic_mover')
        
        # --- 1. Define the Parameter for Speed Control ---
        # This descriptor tells rqt to make a slider from 0.5 to 10.0
        speed_descriptor = ParameterDescriptor(
            description='Time in seconds to complete the move (Lower = Faster)',
            floating_point_range=[FloatingPointRange(from_value=0.5, to_value=10.0, step=0.1)]
        )
        
        # Declare the parameter with a default of 4.0 seconds
        self.declare_parameter('move_duration', 4.0, speed_descriptor)

        # --- 2. Setup the Action Client ---
        # Using the standard Gazebo controller topic
        # self._action_client = ActionClient(
        #     self, 
        #     FollowJointTrajectory, 
        #     '/joint_trajectory_controller/follow_joint_trajectory'
        # )

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

        self.get_logger().info("Waiting for action server: /joint_trajectory_controller/follow_joint_trajectory")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server found! Ready to move.")

    def send_goal(self, joint_angles):
        # --- 3. Get the Current Speed from the Parameter ---
        # This updates every time we call send_goal, allowing real-time adjustment
        duration_sec = self.get_parameter('move_duration').value
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in joint_angles]
        
        # Convert the float duration (e.g., 2.5) into Seconds and Nanoseconds
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f"Moving to target in {duration_sec} seconds...")
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CyclicMover()

    # --- 4. Define the 3 Waypoints (Radians) ---
    points = [
        [-1.4356, -1.6390, -2.1954, 0.8018, 1.3170, 0.0065],       # Point 1: Home/Upright
        [-1.4356, -1.6390, -2.4062, 0.8017, 1.3170, 0.0065],        # Point 2: Reach Forward
        [-1.4356, -1.6390, -2.0994, 0.8018, 1.3170, 0.0065]       # Point 3: Reach Side
    ]

    try:
        while rclpy.ok():
            for i, target in enumerate(points):
                # Send the goal
                future = node.send_goal(target)
                
                # Wait for the server to accept the goal
                rclpy.spin_until_future_complete(node, future)
                goal_handle = future.result()
                
                if not goal_handle.accepted:
                    node.get_logger().error("Goal rejected")
                    break
                
                # Wait for the robot to actually finish moving
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(node, result_future)
                
                # Small pause between moves
                time.sleep(0.5)

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()