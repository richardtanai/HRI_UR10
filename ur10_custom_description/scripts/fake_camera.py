#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import numpy as np
import time

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        self.image_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 10)
        
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        
        # Generate a static pattern (color bars)
        self.image_data = self.generate_pattern()
        
    def generate_pattern(self):
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # Color bars
        step = self.width // 3
        img[:, :step] = [255, 0, 0]      # Red
        img[:, step:2*step] = [0, 255, 0] # Green
        img[:, 2*step:] = [0, 0, 255]    # Blue
        return img.tobytes()

    def timer_callback(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = self.height
        msg.width = self.width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = self.width * 3
        
        # Add some noise to make it "live"
        # For efficiency, we just modify the static buffer slightly or just publish static
        # Let's just publish static for speed, maybe shift it?
        # shift = int(time.time() * 10) % self.width
        # msg.data = self.image_data # + noise
        
        # To show it's alive, let's flash a pixel or something
        # Actually static is fine for debug.
        msg.data = self.image_data
        
        self.image_pub.publish(msg)
        
        # Camera Info
        info = CameraInfo()
        info.header = msg.header
        info.height = self.height
        info.width = self.width
        info.distortion_model = "plumb_bob"
        # Dummy intrinsics
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info.p = [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.info_pub.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
