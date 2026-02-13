import rclpy
import threading
import sys
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RobotRecorder(Node):
    def __init__(self):
        super().__init__('robot_recorder')
        
        # Subscribe to the joint states topic
        # This topic publishes the angle of every joint in real-time
        self.sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.listener_callback, 
            10
        )
        
        self.latest_joints = None
        
        # UR10 Standard Joint Order
        # We must enforce this order so the values match your mover script
        self.target_order = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        self.get_logger().info("Recorder ready. Waiting for joint states...")

    def listener_callback(self, msg):
        # Map the incoming joint names to their positions
        # (Because ROS doesn't always send them in the same order)
        data_map = dict(zip(msg.name, msg.position))
        
        try:
            # Reorder the values to match [Pan, Lift, Elbow, Wrist1, Wrist2, Wrist3]
            ordered_joints = [data_map[name] for name in self.target_order]
            self.latest_joints = ordered_joints
        except KeyError:
            # This happens if the message doesn't contain all UR joints yet
            pass

    def get_current_position(self):
        return self.latest_joints

def main(args=None):
    rclpy.init(args=args)
    node = RobotRecorder()

    # Run ROS in a separate thread so input() doesn't block it
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n" + "="*40)
    print(" UR10 POSITION RECORDER")
    print(" Move the robot in RViz/Gazebo, then:")
    print(" [ENTER] -> Save current position")
    print(" [EXIT]  -> Quit")
    print("="*40 + "\n")

    output_file = "recorded_positions.txt"

    try:
        while rclpy.ok():
            user_input = input("Press ENTER to save (or type 'exit'): ")
            
            if user_input.strip().lower() == 'exit':
                break

            current_joints = node.get_current_position()

            if current_joints is not None:
                # Format as a Python list string: [0.00, -1.57, ...]
                formatted_str = "[" + ", ".join([f"{x:.4f}" for x in current_joints]) + "]"
                
                print(f"Captured: {formatted_str}")
                
                # Append to file
                with open(output_file, "a") as f:
                    f.write(formatted_str + "\n")
                print(f"Saved to {output_file}")
            else:
                print("⚠️  Waiting for joint data... (Is the simulation running?)")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()