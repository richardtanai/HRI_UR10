#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger

from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox, QLineEdit
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from collections import deque


class ArduinoRqtMonitorWidget(QWidget):
    def __init__(self, plugin, plugin_context):
        super(ArduinoRqtMonitorWidget, self).__init__()
        self.setObjectName('ArduinoRqtMonitorWidget')
        
        # Initialize ROS2 node
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('arduino_rqt_monitor')
        
        # Data storage
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        self.weights = deque(maxlen=self.max_points)
        self.start_time = None
        self.current_weight = 0.0
        self.current_led_status = "UNKNOWN"
        self.current_state = "UNKNOWN"
        
        # ROS2 Subscriptions
        self.weight_sub = self.node.create_subscription(
            Float32, '/arduino/weight', self.weight_callback, 10)
        self.led_status_sub = self.node.create_subscription(
            String, '/arduino/led_color_status', self.led_status_callback, 10)
        self.state_sub = self.node.create_subscription(
            String, '/arduino/state', self.state_callback, 10)
            
        # Publishers
        self.duration_pub = self.node.create_publisher(String, '/aruco_sequence/durations', 10)
        
        # Service clients
        self.stop_client = self.node.create_client(Trigger, '/arduino/stop')
        self.tare_client = self.node.create_client(Trigger, '/arduino/tare')
        
        # Setup UI
        self.setup_ui()
        
        # Timer for ROS2 spinning and UI updates
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(10)  # 100Hz
        
        # Timer for plot updates
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)  # 10Hz

    def setup_ui(self):
        main_layout = QVBoxLayout()
        
        # Title
        title = QLabel("<h2>Arduino Monitor Dashboard</h2>")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # Status Display Group
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        # Weight Display
        weight_layout = QHBoxLayout()
        weight_layout.addWidget(QLabel("<b>Weight:</b>"))
        self.weight_label = QLabel("0.0 g")
        self.weight_label.setStyleSheet("font-size: 18px; color: #00ff00;")
        weight_layout.addWidget(self.weight_label)
        weight_layout.addStretch()
        status_layout.addLayout(weight_layout)
        
        # LED Status Display
        led_layout = QHBoxLayout()
        led_layout.addWidget(QLabel("<b>LED Status:</b>"))
        self.led_indicator = QLabel("●")
        self.led_indicator.setStyleSheet("font-size: 24px; color: gray;")
        led_layout.addWidget(self.led_indicator)
        self.led_text = QLabel("UNKNOWN")
        self.led_text.setStyleSheet("font-size: 14px;")
        led_layout.addWidget(self.led_text)
        led_layout.addStretch()
        status_layout.addLayout(led_layout)
        
        # State Display
        state_layout = QHBoxLayout()
        state_layout.addWidget(QLabel("<b>State:</b>"))
        self.state_label = QLabel("UNKNOWN")
        self.state_label.setStyleSheet("font-size: 14px; color: #ffaa00;")
        state_layout.addWidget(self.state_label)
        state_layout.addStretch()
        status_layout.addLayout(state_layout)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # Custom Sequence Group
        seq_group = QGroupBox("Custom Sequence")
        seq_layout = QHBoxLayout()
        
        self.duration_input = QLineEdit()
        self.duration_input.setPlaceholderText("Enter durations e.g. 1, 2, 1, 2")
        seq_layout.addWidget(self.duration_input)
        
        self.set_seq_btn = QPushButton("Set Durations")
        self.set_seq_btn.clicked.connect(self.set_custom_sequence)
        self.set_seq_btn.setStyleSheet("background-color: #555555; color: white; padding: 5px;")
        seq_layout.addWidget(self.set_seq_btn)
        
        seq_group.setLayout(seq_layout)
        main_layout.addWidget(seq_group)
        
        # Weight Chart
        chart_group = QGroupBox("Weight Chart")
        chart_layout = QVBoxLayout()
        
        self.figure = Figure(figsize=(8, 4))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Weight (g)')
        self.ax.set_title('Real-time Weight Data')
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        
        chart_layout.addWidget(self.canvas)
        chart_group.setLayout(chart_layout)
        main_layout.addWidget(chart_group)
        
        # Control Buttons Group
        control_group = QGroupBox("Controls")
        control_layout = QHBoxLayout()
        
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.call_stop)
        self.stop_btn.setStyleSheet("background-color: #cc0000; color: white; padding: 10px;")
        control_layout.addWidget(self.stop_btn)
        
        self.tare_btn = QPushButton("Tare")
        self.tare_btn.clicked.connect(self.call_tare)
        self.tare_btn.setStyleSheet("background-color: #00aa00; color: white; padding: 10px;")
        control_layout.addWidget(self.tare_btn)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        self.setLayout(main_layout)

    def weight_callback(self, msg):
        self.current_weight = msg.data
        if self.start_time is None:
            self.start_time = self.node.get_clock().now().nanoseconds / 1e9
        
        current_time = (self.node.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.times.append(current_time)
        self.weights.append(self.current_weight)
        
        # Update weight label
        self.weight_label.setText(f"{self.current_weight:.2f} g")

    def led_status_callback(self, msg):
        self.current_led_status = msg.data
        self.led_text.setText(self.current_led_status)
        
        # Update LED indicator color
        if self.current_led_status == "BLUE":
            self.led_indicator.setStyleSheet("font-size: 24px; color: blue;")
        elif self.current_led_status == "RED":
            self.led_indicator.setStyleSheet("font-size: 24px; color: red;")
        else:  # OFF or UNKNOWN
            self.led_indicator.setStyleSheet("font-size: 24px; color: gray;")

    def state_callback(self, msg):
        self.current_state = msg.data
        self.state_label.setText(self.current_state)

    def update_plot(self):
        if len(self.times) > 0:
            self.line.set_data(list(self.times), list(self.weights))
            self.ax.relim()
            self.ax.autoscale_view()
            
            # Keep last 10 seconds visible
            if self.times[-1] > 10:
                self.ax.set_xlim(self.times[-1] - 10, self.times[-1] + 0.5)
            
            self.canvas.draw()
            
    def set_custom_sequence(self):
        text = self.duration_input.text()
        if text:
            msg = String()
            msg.data = text
            self.duration_pub.publish(msg)
            self.node.get_logger().info(f"Published sequence: {text}")

    def call_stop(self):
        self.call_service(self.stop_client, "Stop")

    def call_tare(self):
        self.call_service(self.tare_client, "Tare")

    def call_service(self, client, service_name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(f'{service_name} service not available')
            return
        
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.service_response_callback(f, service_name))

    def service_response_callback(self, future, service_name):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f'{service_name}: {response.message}')
            else:
                self.node.get_logger().warn(f'{service_name} failed: {response.message}')
        except Exception as e:
            self.node.get_logger().error(f'{service_name} call failed: {str(e)}')

    def ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def shutdown(self):
        self.ros_timer.stop()
        self.plot_timer.stop()
        self.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
