import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import message_filters
import math
import cv2
import numpy as np
from ultralytics import YOLO

# --- 1. One Euro Filter Class (For Smoothing) ---
class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def __call__(self, t, x):
        t_e = t - self.t_prev
        if t_e <= 0.0: return self.x_prev
        
        dx = (x - self.x_prev) / t_e
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

# --- 2. Main ROS Node ---
class HumanSafetyNode(Node):
    def __init__(self):
        super().__init__('human_safety_node')

        # --- Parameters ---
        self.declare_parameter('safety_distance', 0.6) # Meters
        self.declare_parameter('confidence_threshold', 0.65) # Detection confidence
        self.declare_parameter('filter_min_cutoff', 1.0) # One Euro Filter responsiveness
        self.declare_parameter('filter_beta', 0.007) # One Euro Filter speed adaptation
        
        self.safety_limit = self.get_parameter('safety_distance').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value

        # --- AI Model ---
        self.get_logger().info("Loading YOLOv8 Pose Model...")
        self.model = YOLO("yolov8n-pose.pt")
        self.bridge = CvBridge()

        # --- Camera Intrinsics ---
        self.fx, self.fy, self.cx, self.cy = 0.0, 0.0, 0.0, 0.0
        self.camera_frame = "camera_link"

        # --- Filtering Storage ---
        # Format: { joint_index: {'filters': [FilterX, FilterY, FilterZ], 'last_seen': timestamp} }
        self.filters = {}
        self.filter_timeout = 1.0  # Keep filters alive for 1 second after last detection 

        # --- Skeleton Links (Indices for COCO) ---
        self.skeleton_links = [
            (5, 7), (7, 9),       # Left Arm
            (6, 8), (8, 10),      # Right Arm
            (5, 6), (5, 11), (6, 12), # Torso
            (11, 13), (13, 15),   # Left Leg
            (12, 14), (14, 16),   # Right Leg
            (11, 12)              # Hips
        ]

        # --- Subscribers (Synchronized) ---
        # Adjust topics to match your camera!
        # self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        # self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        # self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')

        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], 
            queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.callback)

        # --- Publishers ---
        self.marker_pub = self.create_publisher(MarkerArray, '/human_pose/markers', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety/emergency_stop', 10) # True = STOP

        self.get_logger().info("Human Safety Node Running. Waiting for sync frames...")

    def callback(self, rgb_msg, depth_msg, info_msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        # 1. Update Intrinsics (Once)
        if self.fx == 0.0:
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.camera_frame = rgb_msg.header.frame_id

        # 2. Convert Images
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 3. AI Inference
        results = self.model(cv_rgb, verbose=False)
        
        marker_array = MarkerArray()
        
        # First, send a DELETE_ALL marker to clear old markers (prevents flickering)
        delete_marker = Marker()
        delete_marker.header.frame_id = self.camera_frame
        delete_marker.header.stamp = rgb_msg.header.stamp
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        points_3d = {} # Store for drawing lines
        stop_triggered = False

        if results[0].keypoints is not None and len(results[0].keypoints.data) > 0:
            # Get first person
            keypoints = results[0].keypoints.data[0] # (17, 3)
            
            # --- PROCESS JOINTS ---
            for i, kpt in enumerate(keypoints):
                conf = kpt[2]
                if conf < self.confidence_threshold: continue

                u, v = int(kpt[0]), int(kpt[1])
                
                # Check image bounds
                if u < 0 or u >= cv_depth.shape[1] or v < 0 or v >= cv_depth.shape[0]:
                    continue

                # Get Depth
                depth_raw = cv_depth[v, u]
                if depth_raw == 0: continue # Invalid depth
                z_raw = depth_raw / 1000.0 # Convert mm to meters

                # Project to 3D (Raw)
                x_raw = (u - self.cx) * z_raw / self.fx
                y_raw = (v - self.cy) * z_raw / self.fy

                # --- APPLY FILTER ---
                if i not in self.filters:
                    min_cutoff = self.get_parameter('filter_min_cutoff').value
                    beta = self.get_parameter('filter_beta').value
                    self.filters[i] = {
                        'filters': [
                            OneEuroFilter(current_time, x_raw, min_cutoff=min_cutoff, beta=beta),
                            OneEuroFilter(current_time, y_raw, min_cutoff=min_cutoff, beta=beta),
                            OneEuroFilter(current_time, z_raw, min_cutoff=min_cutoff, beta=beta)
                        ],
                        'last_seen': current_time
                    }
                
                # Update last seen timestamp
                self.filters[i]['last_seen'] = current_time
                
                # Apply smoothing
                x = self.filters[i]['filters'][0](current_time, x_raw)
                y = self.filters[i]['filters'][1](current_time, y_raw)
                z = self.filters[i]['filters'][2](current_time, z_raw)

                points_3d[i] = (x, y, z)

                # --- SAFETY LOGIC ---
                # Check wrists (9=Left, 10=Right)
                if i == 9 or i == 10:
                    # If wrist is closer than parameter limit
                    if z < self.safety_limit: 
                        stop_triggered = True

                # Create Marker
                sphere = Marker()
                sphere.header.frame_id = self.camera_frame
                sphere.header.stamp = rgb_msg.header.stamp
                sphere.ns = "joints"
                sphere.id = i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = x
                sphere.pose.position.y = y
                sphere.pose.position.z = z
                sphere.scale.x = 0.05
                sphere.scale.y = 0.05
                sphere.scale.z = 0.05
                sphere.lifetime.sec = 0  # 0 means forever (until deleted)
                sphere.lifetime.nanosec = 200000000  # 0.2 seconds
                # Color logic: Red if dangerous, Green if safe
                if (i == 9 or i == 10) and z < self.safety_limit:
                    sphere.color.r, sphere.color.g, sphere.color.b = 1.0, 0.0, 0.0
                else:
                    sphere.color.r, sphere.color.g, sphere.color.b = 0.0, 1.0, 0.0
                sphere.color.a = 1.0
                marker_array.markers.append(sphere)

            # --- DRAW SKELETON LINES ---
            line_marker = Marker()
            line_marker.header.frame_id = self.camera_frame
            line_marker.header.stamp = rgb_msg.header.stamp
            line_marker.ns = "skeleton"
            line_marker.id = 100
            line_marker.type = Marker.LINE_LIST
            line_marker.scale.x = 0.01
            line_marker.lifetime.sec = 0
            line_marker.lifetime.nanosec = 200000000  # 0.2 seconds
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            for link in self.skeleton_links:
                if link[0] in points_3d and link[1] in points_3d:
                    p1 = Point()
                    p1.x, p1.y, p1.z = points_3d[link[0]]
                    p2 = Point()
                    p2.x, p2.y, p2.z = points_3d[link[1]]
                    line_marker.points.append(p1)
                    line_marker.points.append(p2)
            
            if len(line_marker.points) > 0:
                marker_array.markers.append(line_marker)

        # Publish Results
        self.marker_pub.publish(marker_array)
        
        # Publish Safety Stop (True if dangerous, False if safe)
        self.safety_pub.publish(Bool(data=stop_triggered))
        
        if stop_triggered:
            self.get_logger().warn("SAFETY VIOLATION: HAND TOO CLOSE!", throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = HumanSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()