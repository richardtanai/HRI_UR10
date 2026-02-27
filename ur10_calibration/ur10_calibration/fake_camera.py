#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.info_publisher_ = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info("Fake Camera Node Started")

    def timer_callback(self):
        # Create a dummy image (checkerboard or just noise)
        img = np.zeros((480, 640, 3), np.uint8)
        # Draw a circle
        cv2.circle(img, (320, 240), 50, (0, 255, 0), -1)
        
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self.publisher_.publish(msg)
        
        # Publish Camera Info (Dummy)
        info = CameraInfo()
        info.header = msg.header
        info.height = 480
        info.width = 640
        self.info_publisher_.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
