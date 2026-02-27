#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ur_msgs.msg import IOStates
try:
    from ur_dashboard_msgs.msg import SafetyMode
except ImportError:
    SafetyMode = None

class EstopMonitor(Node):
    def __init__(self):
        super().__init__('estop_monitor')

        # Publishers
        self.hard_estop_pub = self.create_publisher(Bool, '/hard_estop', 10)
        self.soft_estop_pub = self.create_publisher(Bool, '/soft_estop', 10)

        # Subscribers
        if SafetyMode:
            self.safety_sub = self.create_subscription(
                SafetyMode, 
                '/safety_mode', 
                self.safety_callback, 
                10
            )
        else:
            self.get_logger().error("ur_dashboard_msgs could not be imported. Hard ESTOP monitoring disabled.")

        self.io_states_sub = self.create_subscription(
            IOStates, 
            '/io_and_status_controller/io_states', 
            self.io_states_callback, 
            10
        )

        # State storage to only publish on change
        self.hard_estop_active = False
        self.soft_estop_active = False

        self.get_logger().info("ESTOP Monitor Node Started. Publishing to /hard_estop and /soft_estop")

    def safety_callback(self, msg):
        # Modes 6 (SYSTEM_EMERGENCY_STOP) and 7 (ROBOT_EMERGENCY_STOP) mean Hard ESTOP
        is_hard_estop = (msg.mode in [6, 7])
        
        if is_hard_estop != self.hard_estop_active:
            self.hard_estop_active = is_hard_estop
            if is_hard_estop:
                self.get_logger().warn("HARD ESTOP DETECTED! (Pendant Button Pressed)")
            else:
                self.get_logger().info("Hard ESTOP Cleared.")
                
        # Publish current state
        bool_msg = Bool()
        bool_msg.data = self.hard_estop_active
        self.hard_estop_pub.publish(bool_msg)

    def io_states_callback(self, msg):
        for digital_in in msg.digital_in_states:
            # We are monitoring pin 3 for the Soft ESTOP
            if digital_in.pin == 3:
                # NC state: goes Low (False) when pressed
                is_soft_estop = not digital_in.state
                
                if is_soft_estop != self.soft_estop_active:
                    self.soft_estop_active = is_soft_estop
                    if is_soft_estop:
                        self.get_logger().warn("SOFT ESTOP DETECTED! (Digital Input Button Pressed)")
                    else:
                        self.get_logger().info("Soft ESTOP Cleared.")
                
                # Publish current state
                bool_msg = Bool()
                bool_msg.data = self.soft_estop_active
                self.soft_estop_pub.publish(bool_msg)

def main(args=None):
    rclpy.init(args=args)
    node = EstopMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
