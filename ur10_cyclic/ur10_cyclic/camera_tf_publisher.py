#!/usr/bin/env python3
"""
Static TF Publisher: Connects camera_link to robot base_link
This bridges the camera TF tree with the robot TF tree.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr  # w
    q[1] = cy * cp * sr - sy * sp * cr  # x
    q[2] = sy * cp * sr + cy * sp * cr  # y
    q[3] = sy * cp * cr - cy * sp * sr  # z

    return q


class CameraTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        
        # Declare parameters for camera position/orientation
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_link')
        self.declare_parameter('x', 0.5)  # 0.5m in front of robot base
        self.declare_parameter('y', 0.0)  # centered
        self.declare_parameter('z', 0.5)  # 0.5m above robot base
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        
        # Get parameters
        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        roll = self.get_parameter('roll').value
        pitch = self.get_parameter('pitch').value
        yaw = self.get_parameter('yaw').value
        
        # Create static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create transform
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = parent_frame
        static_transform.child_frame_id = child_frame
        
        # Set translation
        static_transform.transform.translation.x = x
        static_transform.transform.translation.y = y
        static_transform.transform.translation.z = z
        
        # Set rotation (convert euler to quaternion)
        q = euler_to_quaternion(roll, pitch, yaw)
        static_transform.transform.rotation.x = q[1]
        static_transform.transform.rotation.y = q[2]
        static_transform.transform.rotation.z = q[3]
        static_transform.transform.rotation.w = q[0]
        
        # Publish static transform
        self.tf_broadcaster.sendTransform(static_transform)
        
        self.get_logger().info(f'Publishing static transform: {parent_frame} -> {child_frame}')
        self.get_logger().info(f'Position: x={x}, y={y}, z={z}')
        self.get_logger().info(f'Orientation: roll={roll}, pitch={pitch}, yaw={yaw}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
