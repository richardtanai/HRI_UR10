#!/usr/bin/env python3

import sys
import os
import signal
import subprocess
import threading
import yaml
from datetime import datetime
from collections import deque

import glob
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QGroupBox, QTextEdit, QCheckBox,
    QSplitter, QFrame, QGridLayout, QFileDialog, QComboBox, QSpinBox,
    QInputDialog, QListWidget, QListWidgetItem
)
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt5.QtGui import QColor, QPalette

# Matplotlib for plotting
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, Float32, Empty
from std_srvs.srv import Trigger
from ur_msgs.msg import IOStates
try:
    from ur_dashboard_msgs.msg import SafetyMode
except ImportError:
    SafetyMode = None

class RosNode(Node):
    def __init__(self):
        super().__init__('launch_gui_monitor')
        
        # Data Buffers for Plotting
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        self.weights = deque(maxlen=self.max_points)
        self.start_time = None
        self.current_weight = 0.0
        self.current_led_status = "UNKNOWN"
        self.current_led_state = "UNKNOWN"
        self.current_state = "UNKNOWN"
        self.last_arduino_msg = 0 # Fix crash
        self.soft_estop_active = False
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.camera_callback, 10
        )
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_callback, 10
        )
        self.weight_sub = self.create_subscription(
            Float32, '/arduino/weight', self.weight_callback, 10
        )
        self.led_status_sub = self.create_subscription(
            String, '/arduino/led_color_status', self.led_status_callback, 10
        )
        self.led_state_sub = self.create_subscription(
            String, '/arduino/led_color_cmd', self.led_state_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, '/arduino/state', self.state_callback, 10
        )
        self.perf_robot_sub = self.create_subscription(
            Empty, '/perfect_timing_robot', self.perf_robot_callback, 10
        )
        self.perf_human_sub = self.create_subscription(
            Empty, '/perfect_timing_human', self.perf_human_callback, 10
        )
        self.perfect_timing_robot_flag = False
        self.perfect_timing_human_flag = False
        
        # Publishers
        self.duration_pub = self.create_publisher(String, '/aruco_sequence/durations', 10)
        self.robot_seq_pub = self.create_publisher(String, '/robot_sequence/durations', 10)
        self.robot_seq_fb_pub = self.create_publisher(String, '/robot_sequence/durations_feedback', 10)
        self.series_pub = self.create_publisher(String, '/robot_sequence/series', 10)
        self.series_fb_pub = self.create_publisher(String, '/robot_sequence/series_feedback', 10)
        self.led_state_pub = self.create_publisher(String, '/arduino/led_color_cmd', 10)
        
        # Service Clients
        self.stop_client = self.create_client(Trigger, '/arduino/stop')
        self.tare_client = self.create_client(Trigger, '/arduino/tare')
        self.seq_start_client = self.create_client(Trigger, '/robot_sequence/move_to_start')
        
        # Safety Recovery Clients
        self.unlock_client = self.create_client(Trigger, '/dashboard_client/unlock_protective_stop')
        self.power_on_client = self.create_client(Trigger, '/dashboard_client/power_on')
        self.brake_release_client = self.create_client(Trigger, '/dashboard_client/brake_release')
        self.play_client = self.create_client(Trigger, '/dashboard_client/play')
        
        # Safety Sub
        if SafetyMode:
            self.safety_sub = self.create_subscription(SafetyMode, '/safety_mode', self.safety_callback, 10)
            
        self.io_states_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_states_callback, 10)
        self.script_command_pub = self.create_publisher(String, '/urscript_interface/script_command', 10)
        self.dashboard_stop_client = self.create_client(Trigger, '/dashboard_client/stop')
        
        self.current_safety_mode = "UNKNOWN"
        self.is_estop = False
        self.sequence_status = "IDLE"  # IDLE, RUNNING, COMPLETED
        
        # Subscribe to sequencer status for series auto-advance
        self.seq_status_sub = self.create_subscription(
            String, '/robot_sequence/status', self.seq_status_callback, 10
        )
        
        self.robot_active = False
        self.camera_active = False
        self.handeye_active = False
        self.arduino_active = False
        
        self.last_robot_msg = 0
        self.last_camera_msg = 0
        
        # Timers to reset status (using wall clock for GUI status logic)
        self.check_timer = self.create_timer(1.0, self.check_timeouts)

    def joint_state_callback(self, msg):
        self.last_robot_msg = self.get_clock().now().nanoseconds
        self.robot_active = True

    def camera_callback(self, msg):
        self.last_camera_msg = self.get_clock().now().nanoseconds
        self.camera_active = True

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if 'camera' in transform.child_frame_id or 'calibration' in transform.child_frame_id:
                self.handeye_active = True

    def weight_callback(self, msg):
        self.last_arduino_msg = self.get_clock().now().nanoseconds
        self.arduino_active = True
        self.current_weight = msg.data
        
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.times.append(current_time)
        self.weights.append(self.current_weight)

    def led_status_callback(self, msg):
        self.current_led_status = msg.data

    def led_state_callback(self, msg):
        self.current_led_state = msg.data

    def state_callback(self, msg):
        self.current_state = msg.data

    def perf_robot_callback(self, msg):
        self.get_logger().info("--- PERFECT TIMING ROBOT ---")
        self.perfect_timing_robot_flag = True

    def perf_human_callback(self, msg):
        self.get_logger().info("--- PERFECT TIMING HUMAN ---")
        self.perfect_timing_human_flag = True

    def safety_callback(self, msg):
        # Map mode to string
        modes = {
            1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY",
            5: "SAFEGUARD_STOP", 6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP",
            8: "VIOLATION", 9: "FAULT"
        }
        self.current_safety_mode = modes.get(msg.mode, "UNKNOWN")
        if msg.mode in [6, 7]: # Estop
            self.is_estop = True
        else:
            self.is_estop = False

    def io_states_callback(self, msg):
        for digital_in in msg.digital_in_states:
            if digital_in.pin == 3:
                # NC state: goes Low (False) when pressed
                if not digital_in.state and not self.soft_estop_active:
                    self.soft_estop_active = True
                    self.get_logger().error("SOFT ESTOP PRESSED! Interrupting sequence and commanding stopj(2.0)...")
                    
                    stop_msg = String()
                    stop_msg.data = "stopj(2.0)"
                    self.script_command_pub.publish(stop_msg)
                elif digital_in.state and self.soft_estop_active:
                    self.soft_estop_active = False
                    self.get_logger().info("Soft Estop released.")

    def check_timeouts(self):
        now = self.get_clock().now().nanoseconds
        if now - self.last_robot_msg > 2e9: self.robot_active = False
        if now - self.last_camera_msg > 2e9: self.camera_active = False
        if now - self.last_arduino_msg > 2e9: self.arduino_active = False

    def seq_status_callback(self, msg):
        self.sequence_status = msg.data

    def call_service_nonblocking(self, client, name):
        if not client.service_is_ready():
            return False, f"{name} service not ready"
        
        req = Trigger.Request()
        future = client.call_async(req)
        return True, f"Called {name}"

class LaunchManager(QObject):
    log_signal = pyqtSignal(str)
    process_finished_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.processes = {}

    def launch(self, name, command):
        if name in self.processes:
            self.log_signal.emit(f"Process {name} is already running.")
            return

        self.log_signal.emit(f"Starting {name}...\nCommand: {command}")
        thread = threading.Thread(target=self._run_process, args=(name, command))
        thread.start()

    def _run_process(self, name, command):
        process = subprocess.Popen(
            command, 
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.STDOUT, 
            preexec_fn=os.setsid
        )
        self.processes[name] = process
        while True:
            line = process.stdout.readline()
            if not line: break
            try:
                line.decode('utf-8').strip()
            except: pass
        
        process.wait()
        if name in self.processes: del self.processes[name]
        self.process_finished_signal.emit(name)
        self.log_signal.emit(f"Process {name} finished.")

    def stop(self, name):
        if name in self.processes:
            self.log_signal.emit(f"Stopping {name}...")
            os.killpg(os.getpgid(self.processes[name].pid), signal.SIGTERM)
        else:
            self.log_signal.emit(f"Process {name} is not running.")

class MainWindow(QMainWindow):
    def __init__(self, ros_node, launch_manager):
        super().__init__()
        self.ros_node = ros_node
        self.launch_manager = launch_manager
        self.bag_file = ""
        
        self.setWindowTitle("UR10 Integrated Launch Control")
        self.resize(1000, 800)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Header
        title_label = QLabel("UR10 System Control Center")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 20px; font-weight: bold; margin-bottom: 10px;")
        main_layout.addWidget(title_label)
        
        # System Activity Overview
        overview_group = QGroupBox("System Activity Overview")
        overview_layout = QHBoxLayout()
        
        self.ind_robot = QLabel("UR10: INACTIVE")
        self.ind_robot.setAlignment(Qt.AlignCenter)
        self.ind_robot.setStyleSheet("background-color: lightgray; padding: 5px; border-radius: 5px;")
        overview_layout.addWidget(self.ind_robot)

        self.ind_cam = QLabel("Camera: INACTIVE")
        self.ind_cam.setAlignment(Qt.AlignCenter)
        self.ind_cam.setStyleSheet("background-color: lightgray; padding: 5px; border-radius: 5px;")
        overview_layout.addWidget(self.ind_cam)

        self.ind_seq = QLabel("Sequencer: INACTIVE")
        self.ind_seq.setAlignment(Qt.AlignCenter)
        self.ind_seq.setStyleSheet("background-color: lightgray; padding: 5px; border-radius: 5px;")
        overview_layout.addWidget(self.ind_seq)
        
        self.ind_ard = QLabel("Arduino: INACTIVE")
        self.ind_ard.setAlignment(Qt.AlignCenter)
        self.ind_ard.setStyleSheet("background-color: lightgray; padding: 5px; border-radius: 5px;")
        overview_layout.addWidget(self.ind_ard)

        self.ind_he = QLabel("Handeye: INACTIVE")
        self.ind_he.setAlignment(Qt.AlignCenter)
        self.ind_he.setStyleSheet("background-color: lightgray; padding: 5px; border-radius: 5px;")
        overview_layout.addWidget(self.ind_he)
        
        overview_group.setLayout(overview_layout)
        main_layout.addWidget(overview_group)
        
        # Splitter to separate Launch/Status from Arduino Monitor
        splitter = QSplitter(Qt.Horizontal)
        
        # Left Panel: Launch Controls
        left_widget = QWidget()
        left_layout = QGridLayout(left_widget)
        
        # Robot Section
        robot_group = QGroupBox("UR10 Robot")
        robot_layout = QVBoxLayout()
        settings_layout = QHBoxLayout()
        self.robot_ip_input = QLineEdit("192.168.11.100")
        self.robot_ip_input.setPlaceholderText("Robot IP")
        settings_layout.addWidget(QLabel("IP:"))
        settings_layout.addWidget(self.robot_ip_input)
        self.fake_hardware_chk = QCheckBox("Fake")
        self.fake_hardware_chk.setChecked(True)
        settings_layout.addWidget(self.fake_hardware_chk)
        robot_layout.addLayout(settings_layout)
        
        btn_layout = QHBoxLayout()
        self.robot_start_btn = QPushButton("Start Robot")
        self.robot_start_btn.clicked.connect(self.toggle_robot)
        self.robot_start_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        btn_layout.addWidget(self.robot_start_btn)
        
        self.robot_stopj_btn = QPushButton("STOP (stopj)")
        self.robot_stopj_btn.clicked.connect(self.send_stopj)
        self.robot_stopj_btn.setStyleSheet("background-color: #F44336; color: white; font-weight: bold;")
        btn_layout.addWidget(self.robot_stopj_btn)
        
        self.robot_status = QLabel("INACTIVE")
        self.robot_status.setStyleSheet("color: gray; font-weight: bold;")
        btn_layout.addWidget(self.robot_status)
        robot_layout.addLayout(btn_layout)
        robot_group.setLayout(robot_layout)
        left_layout.addWidget(robot_group, 0, 0)

        # Arduino Launch Section
        arduino_group = QGroupBox("Arduino Launch")
        arduino_layout = QVBoxLayout()
        
        # Port Settings
        ard_settings_layout = QHBoxLayout()
        self.arduino_fake_chk = QCheckBox("Fake")
        ard_settings_layout.addWidget(self.arduino_fake_chk)
        
        ard_settings_layout.addWidget(QLabel("Port:"))
        self.arduino_port_dropdown = QComboBox()
        self.scan_ports() # Initial scan
        ard_settings_layout.addWidget(self.arduino_port_dropdown)
        
        self.btn_refresh_ports = QPushButton("Refresh")
        self.btn_refresh_ports.clicked.connect(self.scan_ports)
        ard_settings_layout.addWidget(self.btn_refresh_ports)
        arduino_layout.addLayout(ard_settings_layout)
        
        # Start/Status
        ard_btn_layout = QHBoxLayout()
        self.arduino_start_btn = QPushButton("Start Driver")
        self.arduino_start_btn.clicked.connect(self.toggle_arduino_driver)
        self.arduino_start_btn.setStyleSheet("background-color: #FF9800; color: white;")
        ard_btn_layout.addWidget(self.arduino_start_btn)
        self.arduino_driver_status = QLabel("INACTIVE")
        self.arduino_driver_status.setStyleSheet("color: gray; font-weight: bold;")
        ard_btn_layout.addWidget(self.arduino_driver_status)
        arduino_layout.addLayout(ard_btn_layout)
        
        arduino_group.setLayout(arduino_layout)
        left_layout.addWidget(arduino_group, 0, 1)
        
        # Camera Section
        camera_group = QGroupBox("Realsense Camera")
        cam_layout = QHBoxLayout()
        self.cam_fake_chk = QCheckBox("Fake")
        cam_layout.addWidget(self.cam_fake_chk)
        self.cam_start_btn = QPushButton("Start Camera")
        self.cam_start_btn.clicked.connect(self.toggle_camera)
        self.cam_start_btn.setStyleSheet("background-color: #2196F3; color: white;")
        cam_layout.addWidget(self.cam_start_btn)
        self.cam_status = QLabel("INACTIVE")
        self.cam_status.setStyleSheet("color: gray; font-weight: bold;")
        cam_layout.addWidget(self.cam_status)
        camera_group.setLayout(cam_layout)
        left_layout.addWidget(camera_group, 1, 0)

        # Handeye Section
        handeye_group = QGroupBox("Handeye Calibration")
        he_layout = QVBoxLayout()
        name_layout = QHBoxLayout()
        self.calib_name = QLineEdit("ur10_eob")
        name_layout.addWidget(QLabel("Name:"))
        name_layout.addWidget(self.calib_name)
        he_layout.addLayout(name_layout)
        
        he_btn_layout = QHBoxLayout()
        self.he_start_btn = QPushButton("Publish")
        self.he_start_btn.clicked.connect(self.toggle_handeye)
        self.he_start_btn.setStyleSheet("background-color: #9C27B0; color: white;")
        he_btn_layout.addWidget(self.he_start_btn)
        self.he_status = QLabel("INACTIVE")
        self.he_status.setStyleSheet("color: gray; font-weight: bold;")
        he_btn_layout.addWidget(self.he_status)
        he_layout.addLayout(he_btn_layout)
        handeye_group.setLayout(he_layout)
        left_layout.addWidget(handeye_group, 1, 1)

        # Robot Sequencer Section
        seq_group = QGroupBox("Robot Sequencer")
        seq_layout = QVBoxLayout()
        
        # Node launch
        self.seq_node_btn = QPushButton("Launch Sequencer Node")
        self.seq_node_btn.clicked.connect(self.toggle_sequencer_node)
        seq_layout.addWidget(self.seq_node_btn)
        
        # ── Preset Selector ──
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("Preset:"))
        self.preset_dropdown = QComboBox()
        self.preset_dropdown.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.preset_dropdown.currentIndexChanged.connect(self.apply_preset)
        preset_layout.addWidget(self.preset_dropdown, 1)
        
        self.btn_import_preset = QPushButton("Import")
        self.btn_import_preset.setToolTip("Load presets from a YAML file")
        self.btn_import_preset.setStyleSheet("background-color: #607D8B; color: white;")
        self.btn_import_preset.clicked.connect(self.import_sequence_from_yaml)
        preset_layout.addWidget(self.btn_import_preset)
        
        self.btn_save_preset = QPushButton("Save")
        self.btn_save_preset.setToolTip("Save current settings as a new preset")
        self.btn_save_preset.setStyleSheet("background-color: #795548; color: white;")
        self.btn_save_preset.clicked.connect(self.save_preset_to_yaml)
        preset_layout.addWidget(self.btn_save_preset)
        seq_layout.addLayout(preset_layout)
        
        # LED Settings
        led_layout = QHBoxLayout()
        led_layout.addWidget(QLabel("LED Sync:"))
        self.led_mode_dropdown = QComboBox()
        self.led_mode_dropdown.addItems(["RANDOM", "RED", "BLUE", "OFF"])
        led_layout.addWidget(self.led_mode_dropdown)
        seq_layout.addLayout(led_layout)
        
        # Red Probability Setting
        prob_layout = QHBoxLayout()
        prob_layout.addWidget(QLabel("Red %:"))
        self.seq_red_prob = QSpinBox()
        self.seq_red_prob.setRange(0, 100)
        self.seq_red_prob.setValue(50)
        prob_layout.addWidget(self.seq_red_prob)
        seq_layout.addLayout(prob_layout)
        
        # Input and Execute
        seq_input_layout = QHBoxLayout()
        self.seq_dur_input = QLineEdit("1, 2, 1")
        self.seq_dur_input.setPlaceholderText("Durations (e.g. 1, 2, 1)")
        seq_input_layout.addWidget(self.seq_dur_input)
        seq_layout.addLayout(seq_input_layout)
        
        exec_btn_layout = QHBoxLayout()
        self.seq_exec_btn = QPushButton("Execute (Time-Based)")
        self.seq_exec_btn.clicked.connect(self.execute_sequence)
        self.seq_exec_btn.setStyleSheet("background-color: #009688; color: white;")
        exec_btn_layout.addWidget(self.seq_exec_btn)

        self.seq_exec_fb_btn = QPushButton("Execute (Feedback-Based)")
        self.seq_exec_fb_btn.clicked.connect(self.execute_sequence_feedback)
        self.seq_exec_fb_btn.setStyleSheet("background-color: #00BCD4; color: black;")
        exec_btn_layout.addWidget(self.seq_exec_fb_btn)
        
        seq_layout.addLayout(exec_btn_layout)

        self.seq_start_btn = QPushButton("Move to Start Position")
        self.seq_start_btn.clicked.connect(self.move_to_start)
        self.seq_start_btn.setStyleSheet("background-color: #FF5722; color: white;")
        seq_layout.addWidget(self.seq_start_btn)
        
        # Perfect Timing Indicators
        timing_layout = QHBoxLayout()
        self.ind_timing_robot = QLabel("Robot Hit")
        self.ind_timing_robot.setAlignment(Qt.AlignCenter)
        self.ind_timing_robot.setStyleSheet("background-color: lightgray; color: black; padding: 10px; border-radius: 5px; font-weight: bold;")
        timing_layout.addWidget(self.ind_timing_robot)

        self.ind_timing_human = QLabel("Human Hit")
        self.ind_timing_human.setAlignment(Qt.AlignCenter)
        self.ind_timing_human.setStyleSheet("background-color: lightgray; color: black; padding: 10px; border-radius: 5px; font-weight: bold;")
        timing_layout.addWidget(self.ind_timing_human)

        seq_layout.addLayout(timing_layout)
        
        seq_group.setLayout(seq_layout)
        left_layout.addWidget(seq_group, 2, 0)
        
        # Store preset data and load defaults
        self.preset_data = []       # list of preset dicts
        self.active_preset_file = ""  # path to last-loaded YAML

        # ════════════════════════════════════════════
        # Series Composer Section
        # ════════════════════════════════════════════
        series_group = QGroupBox("Series Composer")
        series_layout = QVBoxLayout()
        
        # Series list
        self.series_list = QListWidget()
        self.series_list.setStyleSheet(
            "background-color: #1e1e1e; color: #d4d4d4; font-family: Monospace; "
            "font-size: 11px; border: 1px solid #444;"
        )
        self.series_list.setMaximumHeight(120)
        series_layout.addWidget(self.series_list)
        
        # Internal data store: list of preset dicts for the series
        self.series_items = []
        self.series_running = False
        self.series_current_index = 0
        
        # Add / Remove row
        series_btn_row1 = QHBoxLayout()
        
        self.btn_series_add = QPushButton("+ Add Preset")
        self.btn_series_add.setToolTip("Append the currently selected preset to the series")
        self.btn_series_add.setStyleSheet("background-color: #4CAF50; color: white;")
        self.btn_series_add.clicked.connect(self.series_add_preset)
        series_btn_row1.addWidget(self.btn_series_add)
        
        self.btn_series_remove = QPushButton("− Remove")
        self.btn_series_remove.setStyleSheet("background-color: #f44336; color: white;")
        self.btn_series_remove.clicked.connect(self.series_remove_selected)
        series_btn_row1.addWidget(self.btn_series_remove)
        
        self.btn_series_up = QPushButton("▲")
        self.btn_series_up.setFixedWidth(32)
        self.btn_series_up.clicked.connect(self.series_move_up)
        series_btn_row1.addWidget(self.btn_series_up)
        
        self.btn_series_down = QPushButton("▼")
        self.btn_series_down.setFixedWidth(32)
        self.btn_series_down.clicked.connect(self.series_move_down)
        series_btn_row1.addWidget(self.btn_series_down)
        
        self.btn_series_clear = QPushButton("Clear")
        self.btn_series_clear.clicked.connect(self.series_clear)
        series_btn_row1.addWidget(self.btn_series_clear)
        
        series_layout.addLayout(series_btn_row1)
        
        # Execute / Stop / Save / Load row
        series_btn_row2 = QHBoxLayout()
        
        self.btn_series_exec = QPushButton("▶ Execute Series")
        self.btn_series_exec.setStyleSheet("background-color: #009688; color: white; font-weight: bold;")
        self.btn_series_exec.clicked.connect(self.series_execute)
        series_btn_row2.addWidget(self.btn_series_exec)
        
        self.btn_series_stop = QPushButton("■ Stop")
        self.btn_series_stop.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.btn_series_stop.clicked.connect(self.series_stop)
        self.btn_series_stop.setEnabled(False)
        series_btn_row2.addWidget(self.btn_series_stop)
        
        self.btn_series_save = QPushButton("Save")
        self.btn_series_save.setToolTip("Save current series to YAML")
        self.btn_series_save.setStyleSheet("background-color: #795548; color: white;")
        self.btn_series_save.clicked.connect(self.series_save_to_yaml)
        series_btn_row2.addWidget(self.btn_series_save)
        
        self.btn_series_load = QPushButton("Load")
        self.btn_series_load.setToolTip("Load a series from YAML")
        self.btn_series_load.setStyleSheet("background-color: #607D8B; color: white;")
        self.btn_series_load.clicked.connect(self.series_load_from_yaml)
        series_btn_row2.addWidget(self.btn_series_load)
        
        series_layout.addLayout(series_btn_row2)
        
        # Series progress label
        self.series_status_label = QLabel("Series: Idle")
        self.series_status_label.setStyleSheet("color: gray; font-weight: bold;")
        series_layout.addWidget(self.series_status_label)
        
        series_group.setLayout(series_layout)
        left_layout.addWidget(series_group, 2, 1, 2, 1)  # span rows 2-3, column 1

        # Visualization Section
        viz_group = QGroupBox("Visualization")
        viz_layout = QHBoxLayout()
        self.viz_live_btn = QPushButton("Live Viz")
        self.viz_live_btn.clicked.connect(self.toggle_live_visualization)
        self.viz_live_btn.setStyleSheet("background-color: #673AB7; color: white;")
        viz_layout.addWidget(self.viz_live_btn)
        
        self.viz_play_btn = QPushButton("Playback Viz")
        self.viz_play_btn.clicked.connect(self.toggle_visualization)
        self.viz_play_btn.setStyleSheet("background-color: #9C27B0; color: white;")
        viz_layout.addWidget(self.viz_play_btn)
        viz_group.setLayout(viz_layout)
        left_layout.addWidget(viz_group, 3, 0)

        # Recording Section
        rec_group = QGroupBox("Recording")
        rec_layout = QVBoxLayout()
        
        dir_layout = QHBoxLayout()
        dir_layout.addWidget(QLabel("Save Path:"))
        self.rec_dir_input = QLineEdit(os.path.join(os.path.expanduser("~"), "ur10_recordings"))
        dir_layout.addWidget(self.rec_dir_input)
        self.rec_dir_btn = QPushButton("Browse...")
        self.rec_dir_btn.clicked.connect(self.select_rec_dir)
        dir_layout.addWidget(self.rec_dir_btn)
        rec_layout.addLayout(dir_layout)

        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Filename:"))
        self.rec_name_input = QLineEdit("ur10_record")
        name_layout.addWidget(self.rec_name_input)
        rec_layout.addLayout(name_layout)

        btn_layout = QHBoxLayout()
        self.rec_btn = QPushButton("Record Data (MCAP)")
        self.rec_btn.clicked.connect(self.toggle_recording)
        self.rec_btn.setStyleSheet("background-color: #E91E63; color: white;")
        btn_layout.addWidget(self.rec_btn)
        self.rec_status = QLabel("INACTIVE")
        self.rec_status.setStyleSheet("color: gray; font-weight: bold;")
        btn_layout.addWidget(self.rec_status)
        rec_layout.addLayout(btn_layout)
        rec_group.setLayout(rec_layout)
        left_layout.addWidget(rec_group, 4, 0)

        # Playback Section
        play_group = QGroupBox("Bag Playback")
        play_layout = QVBoxLayout()
        
        file_layout = QHBoxLayout()
        self.bag_label = QLabel("No file selected")
        self.bag_label.setStyleSheet("color: gray;")
        file_layout.addWidget(self.bag_label)
        self.btn_select_bag = QPushButton("Select File")
        self.btn_select_bag.clicked.connect(self.select_bag_file)
        file_layout.addWidget(self.btn_select_bag)
        play_layout.addLayout(file_layout)
        
        play_btn_layout = QHBoxLayout()
        self.btn_play_bag = QPushButton("Play Bag")
        self.btn_play_bag.clicked.connect(self.toggle_bag_playback)
        self.btn_play_bag.setStyleSheet("background-color: #3F51B5; color: white;")
        play_btn_layout.addWidget(self.btn_play_bag)
        
        self.btn_info_bag = QPushButton("Bag Info")
        self.btn_info_bag.clicked.connect(self.get_bag_info)
        play_btn_layout.addWidget(self.btn_info_bag)
        
        self.play_status = QLabel("INACTIVE")
        self.play_status.setStyleSheet("color: gray; font-weight: bold;")
        play_btn_layout.addWidget(self.play_status)
        play_layout.addLayout(play_btn_layout)
        
        play_group.setLayout(play_layout)
        left_layout.addWidget(play_group, 4, 1)

        # Safety Section
        safe_group = QGroupBox("Safety & Recovery")
        safe_layout = QVBoxLayout()
        
        # Status
        status_layout = QHBoxLayout()
        status_layout.addWidget(QLabel("Safety Mode:"))
        self.safety_label = QLabel("UNKNOWN")
        self.safety_label.setStyleSheet("color: gray; font-weight: bold;")
        status_layout.addWidget(self.safety_label)
        safe_layout.addLayout(status_layout)
        
        # Recovery Buttons
        rec_btn_layout = QHBoxLayout()
        self.btn_unlock = QPushButton("Unlock Stop")
        self.btn_unlock.clicked.connect(lambda: self.call_service(self.ros_node.unlock_client, "Unlock"))
        self.btn_unlock.setStyleSheet("background-color: #FFC107;")
        rec_btn_layout.addWidget(self.btn_unlock)
        
        self.btn_power = QPushButton("Power On")
        self.btn_power.clicked.connect(lambda: self.call_service(self.ros_node.power_on_client, "Power On"))
        self.btn_power.setStyleSheet("background-color: #4CAF50; color: white;")
        rec_btn_layout.addWidget(self.btn_power)

        self.btn_brake = QPushButton("Release Brake")
        self.btn_brake.clicked.connect(lambda: self.call_service(self.ros_node.brake_release_client, "Brake Rel"))
        self.btn_brake.setStyleSheet("background-color: #2196F3; color: white;")
        rec_btn_layout.addWidget(self.btn_brake)
        
        safe_layout.addLayout(rec_btn_layout)
        
        play_btn_layout = QHBoxLayout()
        self.btn_run_prog = QPushButton("Run Program (Play)")
        self.btn_run_prog.clicked.connect(lambda: self.call_service(self.ros_node.play_client, "Play Program"))
        self.btn_run_prog.setStyleSheet("background-color: #8BC34A; color: black; font-weight: bold;")
        play_btn_layout.addWidget(self.btn_run_prog)
        safe_layout.addLayout(play_btn_layout)
        
        safe_group.setLayout(safe_layout)
        left_layout.addWidget(safe_group, 5, 0)

        # Tools Section
        misc_group = QGroupBox("Tools")
        misc_layout = QVBoxLayout()
        self.moveit_btn = QPushButton("Start MoveIt")
        self.moveit_btn.clicked.connect(self.toggle_moveit)
        misc_layout.addWidget(self.moveit_btn)
        misc_group.setLayout(misc_layout)
        left_layout.addWidget(misc_group, 5, 1)
        left_layout.setRowStretch(6, 1)
        
        splitter.addWidget(left_widget)
        
        # Right Panel: Arduino Monitor & Logs
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Arduino Monitor Widget
        monitor_group = QGroupBox("Arduino Monitor")
        mon_layout = QVBoxLayout()
        
        # Status Grid
        status_grid = QGridLayout()
        status_grid.addWidget(QLabel("Weight:"), 0, 0)
        self.val_weight = QLabel("0.0 g")
        self.val_weight.setStyleSheet("font-size: 24px; color: #4CAF50; font-weight: bold;")
        status_grid.addWidget(self.val_weight, 0, 1)
        
        status_grid.addWidget(QLabel("LED Cmd:"), 0, 2)
        self.val_led_cmd = QLabel("●")
        self.val_led_cmd.setStyleSheet("font-size: 24px; color: gray;")
        status_grid.addWidget(self.val_led_cmd, 0, 3)
        self.txt_led_cmd = QLabel("UNKNOWN")
        status_grid.addWidget(self.txt_led_cmd, 0, 4)

        status_grid.addWidget(QLabel("LED Stat:"), 0, 5)
        self.val_led_stat = QLabel("●")
        self.val_led_stat.setStyleSheet("font-size: 24px; color: gray;")
        status_grid.addWidget(self.val_led_stat, 0, 6)
        self.txt_led_stat = QLabel("UNKNOWN")
        status_grid.addWidget(self.txt_led_stat, 0, 7)
        
        status_grid.addWidget(QLabel("State:"), 1, 0)
        self.val_state = QLabel("UNKNOWN")
        self.val_state.setStyleSheet("font-size: 14px; color: #ff9800;")
        status_grid.addWidget(self.val_state, 1, 1, 1, 4)
        mon_layout.addLayout(status_grid)
        
        # Plot
        self.figure = Figure(figsize=(5, 3))
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("Weight (g)")
        self.ax.grid(True)
        self.line, = self.ax.plot([], [], 'b-')
        mon_layout.addWidget(self.toolbar)
        mon_layout.addWidget(self.canvas)
        
        # Controls
        ctrl_layout = QGridLayout()

        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background-color: red; color: white;")
        self.btn_stop.clicked.connect(lambda: self.call_service(self.ros_node.stop_client, "Stop"))
        ctrl_layout.addWidget(self.btn_stop, 0, 0)
        
        self.btn_tare = QPushButton("Tare")
        self.btn_tare.setStyleSheet("background-color: green; color: white;")
        self.btn_tare.clicked.connect(lambda: self.call_service(self.ros_node.tare_client, "Tare"))
        ctrl_layout.addWidget(self.btn_tare, 1, 1)
        mon_layout.addLayout(ctrl_layout)
        
        # Custom Duration
        dur_layout = QHBoxLayout()
        self.dur_input = QLineEdit()
        self.dur_input.setPlaceholderText("Durations (e.g., 1,2,1)")
        dur_layout.addWidget(self.dur_input)
        self.btn_set_dur = QPushButton("Set")
        self.btn_set_dur.clicked.connect(self.set_durations)
        dur_layout.addWidget(self.btn_set_dur)
        mon_layout.addLayout(dur_layout)
        
        monitor_group.setLayout(mon_layout)
        right_layout.addWidget(monitor_group)
        
        # Log Area
        right_layout.addWidget(QLabel("Logs:"))
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setStyleSheet("background-color: #000; color: #0f0; font-family: Monospace;")
        right_layout.addWidget(self.log_area)
        
        splitter.addWidget(right_widget)
        main_layout.addWidget(splitter)
        
        # Signals & State
        self.launch_manager.log_signal.connect(self.log_message)
        self.launch_manager.process_finished_signal.connect(self.process_finished)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100) # 10Hz UI update
        
        self.running_states = {
            'robot': False, 'camera': False, 'handeye': False, 
            'moveit': False, 'recording': False, 'arduino': False,
            'visualization_playback': False, 'visualization_live': False,
            'sequencer': False,
            'playback': False,
            'bag_info': False
        }
        
        # Auto-load default presets from package config
        default_preset_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'sequence_presets.yaml'
        )
        if os.path.isfile(default_preset_path):
            self.load_presets_from_yaml(default_preset_path)

    def log_message(self, msg):
        self.log_area.append(msg)

    def process_finished(self, name):
        self.running_states[name] = False
        self.update_buttons()
        self.log_message(f"Process {name} stopped/finished.")

    def update_buttons(self):
        # Update text/style based on running_states
        self.update_btn(self.robot_start_btn, self.running_states['robot'], "Start Robot", "Stop Robot", "#4CAF50")
        self.update_btn(self.arduino_start_btn, self.running_states['arduino'], "Start Driver", "Stop Driver", "#FF9800")
        self.update_btn(self.cam_start_btn, self.running_states['camera'], "Start Camera", "Stop Camera", "#2196F3")
        self.update_btn(self.he_start_btn, self.running_states['handeye'], "Publish", "Stop", "#9C27B0")
        self.update_btn(self.moveit_btn, self.running_states['moveit'], "Start MoveIt", "Stop MoveIt", "")
        self.update_btn(self.rec_btn, self.running_states['recording'], "Record Data", "Stop Recording", "#E91E63")
        self.update_btn(self.btn_play_bag, self.running_states['playback'], "Play Bag", "Stop Playback", "#3F51B5")
        self.update_btn(self.seq_node_btn, self.running_states['sequencer'], "Launch Sequencer Node", "Stop Sequencer Node", "#FF9800")

    def update_btn(self, btn, running, start_text, stop_text, start_color):
        if running:
            btn.setText(stop_text)
            btn.setStyleSheet("background-color: #f44336; color: white;")
        else:
            btn.setText(start_text)
            if start_color: btn.setStyleSheet(f"background-color: {start_color}; color: white;")
            else: btn.setStyleSheet("")

    def toggle_robot(self):
        if self.running_states['robot']:
            self.launch_manager.stop('robot')
        else:
            ip = self.robot_ip_input.text()
            fake = self.fake_hardware_chk.isChecked()
            if fake: cmd = "ros2 launch ur10_custom_description ur10_custom.launch.py use_fake_hardware:=true"
            else: cmd = f"ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:={ip} launch_rviz:=false description_package:=ur10_custom_description description_file:=ur10_with_fist.urdf.xacro"
            self.launch_manager.launch('robot', cmd)
            self.running_states['robot'] = True
        self.update_buttons()

    def send_stopj(self):
        stop_msg = String()
        stop_msg.data = "stopj(2.0)"
        self.ros_node.script_command_pub.publish(stop_msg)
        self.log_message("Manual stopj(2.0) command sent to robot.")

    def scan_ports(self):
        self.arduino_port_dropdown.clear()
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if not ports:
            self.arduino_port_dropdown.addItem("/dev/ttyACM0") # Default fallback
        else:
            self.arduino_port_dropdown.addItems(sorted(ports))
            
    def toggle_arduino_driver(self):
        if self.running_states['arduino']:
            self.launch_manager.stop('arduino')
        else:
            fake = self.arduino_fake_chk.isChecked()
            port = self.arduino_port_dropdown.currentText()
            # If fake, we use fake monitor launch which launches fake bridge
            # Real: arduino_interface.launch.py -> launches bridge.
            if fake: cmd = f"ros2 launch ur10_cyclic arduino_interface.launch.py fake_hardware:=true port:={port}"
            else: cmd = f"ros2 launch ur10_cyclic arduino_interface.launch.py port:={port}"
            self.launch_manager.launch('arduino', cmd)
            self.running_states['arduino'] = True
        self.update_buttons()

    def toggle_camera(self):
        if self.running_states['camera']:
            self.launch_manager.stop('camera')
        else:
            fake = self.cam_fake_chk.isChecked()
            if fake:
                cmd = "ros2 run ur10_custom_description fake_camera.py"
            else:
                cmd = "ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true rgb_camera.profile:=640x480x30 depth_module.depth_profile:=640x480x30"
            self.launch_manager.launch('camera', cmd)
            self.running_states['camera'] = True
        self.update_buttons()

    def toggle_handeye(self):
        if self.running_states['handeye']:
            self.launch_manager.stop('handeye')
        else:
            name = self.calib_name.text()
            cmd = f"ros2 launch easy_handeye2 publish.launch.py name:={name}"
            self.launch_manager.launch('handeye', cmd)
            self.running_states['handeye'] = True
        self.update_buttons()

    def toggle_moveit(self):
        if self.running_states['moveit']:
            self.launch_manager.stop('moveit')
        else:
            if self.running_states['robot']: # Robot running, launch only moveit config
                 cmd = "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 description_package:=ur10_custom_description description_file:=ur10_with_fist.urdf.xacro launch_rviz:=true"
            else: # Robot not running, launch full stack (sim)
                 cmd = "ros2 launch ur10_custom_description ur10_moveit.launch.py"
            self.launch_manager.launch('moveit', cmd)
            self.running_states['moveit'] = True
        self.update_buttons()

    def toggle_recording(self):
        if self.running_states['recording']:
            self.launch_manager.stop('recording')
        else:
            if not self.running_states['robot']:
                self.log_message("WARNING: Robot Driver is NOT running! Recording might be empty.")
                # We don't block, but we warn.
            
            prefix = self.rec_name_input.text()
            if not prefix: prefix = "ur10_record"
            
            # Create recording directory
            rec_dir = self.rec_dir_input.text()
            if not rec_dir:
                home = os.path.expanduser("~")
                rec_dir = os.path.join(home, "ur10_recordings")
            os.makedirs(rec_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
            bag_name = f"{prefix}_{timestamp}"
            full_path = os.path.join(rec_dir, bag_name)
            
            topics = "/joint_states /tf /tf_static /mid_mount_joint_states /camera/camera/color/image_raw/compressed /camera/camera/aligned_depth_to_color/image_raw/compressedDepth  /camera/camera/color/camera_info /arduino/weight /arduino/led_color_cmd /arduino/led_color_status /arduino/led_value /arduino/state /safety_mode /hard_estop /soft_estop /perfect_timing_robot /perfect_timing_human"
            
            cmd = f"ros2 bag record -s mcap -o {full_path} {topics}"
            self.launch_manager.launch('recording', cmd)
            self.running_states['recording'] = True
            self.log_message(f"Recording started. Saving to: {full_path}")
        self.update_buttons()

    def toggle_visualization(self):
        cmd = "ros2 launch ur10_custom_description view_recording.launch.py"
        self.launch_manager.launch('visualization_playback', cmd)
        self.running_states['visualization_playback'] = True

    def toggle_live_visualization(self):
        cmd = "ros2 launch ur10_custom_description view_live.launch.py"
        self.launch_manager.launch('visualization_live', cmd)
        self.running_states['visualization_live'] = True

    def toggle_sequencer_node(self):
        if self.running_states['sequencer']:
            self.launch_manager.stop('sequencer')
        else:
            cmd = "ros2 run ur10_custom_description robot_sequencer.py"
            self.launch_manager.launch('sequencer', cmd)
            self.running_states['sequencer'] = True
        self.update_buttons()

    def execute_sequence(self):
        text = self.seq_dur_input.text()
        prob = self.seq_red_prob.value()
        if text:
            led_mode = self.led_mode_dropdown.currentText()
            
            msg = String()
            msg.data = f"{text}|{prob}|{led_mode}"
            self.ros_node.robot_seq_pub.publish(msg)
            self.log_message(f"Sent Sequence Durations: {text} | LED Mode: {led_mode} | Red %: {prob}")

    def execute_sequence_feedback(self):
        text = self.seq_dur_input.text()
        prob = self.seq_red_prob.value()
        if text:
            led_mode = self.led_mode_dropdown.currentText()
            
            msg = String()
            msg.data = f"{text}|{prob}|{led_mode}"
            self.ros_node.robot_seq_fb_pub.publish(msg)
            self.log_message(f"Sent Feedback Sequence Durations: {text} | LED Mode: {led_mode} | Red %: {prob}")

    # ── Preset YAML Functions ──────────────────────────────────

    def load_presets_from_yaml(self, filepath):
        """Load presets from a YAML file and populate the dropdown."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            presets = data.get('presets', [])
            if not presets:
                self.log_message(f"No presets found in {filepath}")
                return
            
            self.preset_data = presets
            self.active_preset_file = filepath
            
            # Block signals while repopulating to avoid triggering apply_preset
            self.preset_dropdown.blockSignals(True)
            self.preset_dropdown.clear()
            self.preset_dropdown.addItem("-- Select Preset --")
            for p in presets:
                self.preset_dropdown.addItem(p.get('name', 'Unnamed'))
            self.preset_dropdown.blockSignals(False)
            
            self.log_message(f"Loaded {len(presets)} presets from: {os.path.basename(filepath)}")
        except Exception as e:
            self.log_message(f"Error loading presets: {e}")

    def import_sequence_from_yaml(self):
        """Open a file dialog to pick a preset YAML file."""
        # Default to the package config directory
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        if not os.path.isdir(default_dir):
            default_dir = os.getcwd()
        
        filepath, _ = QFileDialog.getOpenFileName(
            self, 'Import Sequence Presets', default_dir,
            "YAML Files (*.yaml *.yml)"
        )
        if filepath:
            self.load_presets_from_yaml(filepath)

    def apply_preset(self, index):
        """Apply the selected preset to the GUI fields."""
        # index 0 is the placeholder "-- Select Preset --"
        if index <= 0 or index > len(self.preset_data):
            return
        
        preset = self.preset_data[index - 1]
        
        # Durations
        durations = preset.get('durations', '')
        self.seq_dur_input.setText(str(durations))
        
        # LED mode
        led_mode = preset.get('led_mode', 'RANDOM')
        idx = self.led_mode_dropdown.findText(led_mode)
        if idx >= 0:
            self.led_mode_dropdown.setCurrentIndex(idx)
        
        # Red probability
        red_prob = preset.get('red_prob', 50)
        self.seq_red_prob.setValue(int(red_prob))
        
        mode = preset.get('mode', 'time')
        self.log_message(
            f"Preset applied: {preset.get('name')} | "
            f"Durations: {durations} | LED: {led_mode} | "
            f"Red%: {red_prob} | Mode: {mode}"
        )

    def save_preset_to_yaml(self):
        """Save the current GUI settings as a new preset, appended to the active YAML file."""
        # Ask for a preset name
        name, ok = QInputDialog.getText(self, 'Save Preset', 'Preset name:')
        if not ok or not name.strip():
            return
        
        new_preset = {
            'name': name.strip(),
            'durations': self.seq_dur_input.text(),
            'led_mode': self.led_mode_dropdown.currentText(),
            'red_prob': self.seq_red_prob.value(),
            'mode': 'time'
        }
        
        # Determine target file
        if self.active_preset_file and os.path.isfile(self.active_preset_file):
            target = self.active_preset_file
        else:
            target, _ = QFileDialog.getSaveFileName(
                self, 'Save Preset File', os.getcwd(),
                "YAML Files (*.yaml *.yml)"
            )
            if not target:
                return
        
        # Read existing data or create new
        try:
            if os.path.isfile(target):
                with open(target, 'r') as f:
                    data = yaml.safe_load(f) or {}
            else:
                data = {}
            
            if 'presets' not in data:
                data['presets'] = []
            
            data['presets'].append(new_preset)
            
            with open(target, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            
            self.log_message(f"Saved preset '{name}' to {os.path.basename(target)}")
            
            # Reload to reflect the newly saved preset
            self.load_presets_from_yaml(target)
        except Exception as e:
            self.log_message(f"Error saving preset: {e}")

    # ── Series Composer Functions ────────────────────────────────

    def _series_refresh_list(self):
        """Rebuild the QListWidget display from self.series_items."""
        self.series_list.clear()
        for i, item in enumerate(self.series_items):
            prefix = ""
            if self.series_running and i < self.series_current_index:
                prefix = "✓ "
            elif self.series_running and i == self.series_current_index:
                prefix = "▶ "
            else:
                prefix = f"{i+1}. "
            
            mode_tag = "[T]" if item.get('mode', 'time') == 'time' else "[FB]"
            display = f"{prefix}{item.get('name', 'Unnamed')} {mode_tag} — {item.get('durations', '')}"
            self.series_list.addItem(display)

    def series_add_preset(self):
        """Add the currently selected preset to the series."""
        idx = self.preset_dropdown.currentIndex()
        if idx <= 0 or idx > len(self.preset_data):
            self.log_message("Select a preset first before adding to series.")
            return
        
        import copy
        preset = copy.deepcopy(self.preset_data[idx - 1])
        self.series_items.append(preset)
        self._series_refresh_list()
        self.log_message(f"Added '{preset.get('name')}' to series (#{len(self.series_items)})")

    def series_remove_selected(self):
        """Remove the currently selected item from the series."""
        row = self.series_list.currentRow()
        if row >= 0 and row < len(self.series_items):
            removed = self.series_items.pop(row)
            self._series_refresh_list()
            self.log_message(f"Removed '{removed.get('name')}' from series")

    def series_move_up(self):
        """Move the selected series item up by one position."""
        row = self.series_list.currentRow()
        if row > 0:
            self.series_items[row], self.series_items[row-1] = self.series_items[row-1], self.series_items[row]
            self._series_refresh_list()
            self.series_list.setCurrentRow(row - 1)

    def series_move_down(self):
        """Move the selected series item down by one position."""
        row = self.series_list.currentRow()
        if row >= 0 and row < len(self.series_items) - 1:
            self.series_items[row], self.series_items[row+1] = self.series_items[row+1], self.series_items[row]
            self._series_refresh_list()
            self.series_list.setCurrentRow(row + 1)

    def series_clear(self):
        """Clear all items from the series."""
        self.series_items.clear()
        self._series_refresh_list()
        self.log_message("Series cleared.")

    def series_execute(self):
        """Compose the entire series into one flat sequence and send as a single trajectory."""
        if not self.series_items:
            self.log_message("Series is empty. Add presets first.")
            return
        
        import random
        
        # Flatten all presets into one durations list + one colors list
        all_durations = []
        all_colors = []
        has_feedback = False
        
        for item in self.series_items:
            dur_str = str(item.get('durations', ''))
            durations = [float(x.strip()) for x in dur_str.split(',') if x.strip()]
            led_mode = item.get('led_mode', 'RANDOM').upper()
            red_prob = int(item.get('red_prob', 50))
            mode = item.get('mode', 'time')
            if mode == 'feedback':
                has_feedback = True
            
            # Pre-resolve LED color for each cycle in this preset
            for dur in durations:
                if led_mode == 'RANDOM':
                    color = 'red' if random.randint(1, 100) <= red_prob else 'blue'
                elif led_mode == 'RED':
                    color = 'red'
                elif led_mode == 'BLUE':
                    color = 'blue'
                else:
                    color = 'off'
                all_durations.append(dur)
                all_colors.append(color)
        
        # Build the message: "dur1,dur2,...|color1,color2,..."
        dur_str = ','.join(str(d) for d in all_durations)
        color_str = ','.join(all_colors)
        msg = String()
        msg.data = f"{dur_str}|{color_str}"
        
        # Use feedback topic if any preset uses feedback mode
        if has_feedback:
            self.ros_node.series_fb_pub.publish(msg)
        else:
            self.ros_node.series_pub.publish(msg)
        
        # Calculate total time
        total_time = sum(d * 2 for d in all_durations)  # each cycle is 2 moves
        total_cycles = len(all_durations)
        
        # UI state
        self.series_running = True
        self.series_current_index = 0
        self.ros_node.sequence_status = "IDLE"
        self.btn_series_exec.setEnabled(False)
        self.btn_series_stop.setEnabled(True)
        self.series_status_label.setText(f"Series: Running ({total_cycles} cycles, ~{total_time:.0f}s)")
        self.series_status_label.setStyleSheet("color: #009688; font-weight: bold;")
        self._series_refresh_list()
        
        # Log details
        self.log_message(f"=== Series Executing as Single Trajectory ===")
        self.log_message(f"  Total cycles: {total_cycles} | Est. time: {total_time:.1f}s")
        self.log_message(f"  Mode: {'Feedback' if has_feedback else 'Time-Based'}")
        self.log_message(f"  Colors: {all_colors}")

    def _series_finished(self):
        """Called when the entire series has finished."""
        self.series_running = False
        self.btn_series_exec.setEnabled(True)
        self.btn_series_stop.setEnabled(False)
        self.series_status_label.setText("Series: Completed ✓")
        self.series_status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        self._series_refresh_list()
        self.log_message("=== Series Completed ===")

    def series_stop(self):
        """Stop the running series (current sequence will still finish)."""
        if self.series_running:
            self.series_running = False
            self.btn_series_exec.setEnabled(True)
            self.btn_series_stop.setEnabled(False)
            self.series_status_label.setText("Series: Stopped")
            self.series_status_label.setStyleSheet("color: #f44336; font-weight: bold;")
            self._series_refresh_list()
            self.log_message(
                f"Series stopped at step {self.series_current_index + 1}/{len(self.series_items)}. "
                f"Current sequence may still complete."
            )

    def series_save_to_yaml(self):
        """Save the current series composition to a YAML file."""
        if not self.series_items:
            self.log_message("Series is empty, nothing to save.")
            return
        
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        if not os.path.isdir(default_dir):
            default_dir = os.getcwd()
        
        filepath, _ = QFileDialog.getSaveFileName(
            self, 'Save Series', default_dir,
            "YAML Files (*.yaml *.yml)"
        )
        if not filepath:
            return
        
        data = {'series': self.series_items}
        try:
            with open(filepath, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.log_message(f"Series saved ({len(self.series_items)} items) to {os.path.basename(filepath)}")
        except Exception as e:
            self.log_message(f"Error saving series: {e}")

    def series_load_from_yaml(self):
        """Load a series composition from a YAML file."""
        default_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config'
        )
        if not os.path.isdir(default_dir):
            default_dir = os.getcwd()
        
        filepath, _ = QFileDialog.getOpenFileName(
            self, 'Load Series', default_dir,
            "YAML Files (*.yaml *.yml)"
        )
        if not filepath:
            return
        
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            
            items = data.get('series', [])
            if not items:
                self.log_message(f"No series data found in {filepath}")
                return
            
            self.series_items = items
            self._series_refresh_list()
            self.log_message(f"Loaded series ({len(items)} items) from {os.path.basename(filepath)}")
        except Exception as e:
            self.log_message(f"Error loading series: {e}")

    def move_to_start(self):
        self.call_service(self.ros_node.seq_start_client, '/robot_sequence/move_to_start')

    def call_service(self, client, name):
        success, msg = self.ros_node.call_service_nonblocking(client, name)
        self.log_message(msg)

    def set_durations(self):
        text = self.dur_input.text()
        if text:
            msg = String()
            msg.data = text
            self.ros_node.duration_pub.publish(msg)
            self.log_message(f"Set Durations: {text}")

    def select_rec_dir(self):
        dir_path = QFileDialog.getExistingDirectory(self, "Select Recording Directory", self.rec_dir_input.text())
        if dir_path:
            self.rec_dir_input.setText(dir_path)

    def select_bag_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open MCAP File', os.getcwd(), "MCAP Files (*.mcap)")
        if fname:
            self.bag_file = fname
            self.bag_label.setText(os.path.basename(fname))
            self.log_message(f"Selected bag file: {fname}")

    def toggle_bag_playback(self):
        if self.running_states['playback']:
            self.launch_manager.stop('playback')
        else:
            if not self.bag_file:
                self.log_message("No bag file selected!")
                return
            
            # If metadata.yaml exists in parent, use parent directory
            play_path = self.bag_file
            if self.bag_file.endswith(".mcap"):
                parent = os.path.dirname(self.bag_file)
                if os.path.exists(os.path.join(parent, "metadata.yaml")):
                    play_path = parent
            
            # Put --clock at the end to avoid argument parsing issues
            cmd = f"ros2 bag play '{play_path}' --clock"
            self.launch_manager.launch('playback', cmd)
            self.running_states['playback'] = True
            self.log_message(f"Playback started for: {os.path.basename(self.bag_file)}")
        self.update_buttons()

    def get_bag_info(self):
        if not self.bag_file:
            self.log_message("No bag file selected!")
            return
            
        play_path = self.bag_file
        if self.bag_file.endswith(".mcap"):
            parent = os.path.dirname(self.bag_file)
            if os.path.exists(os.path.join(parent, "metadata.yaml")):
                play_path = parent
        
        cmd = f"ros2 bag info '{play_path}'"
        self.log_message(f"Running: {cmd}")
        # Run as a one-off process, output will be captured
        self.launch_manager.launch('bag_info', cmd)

    def update_ui(self):
        # Update Overview Indicators
        self.update_overview_indicator(self.ind_robot, "UR10:", self.ros_node.robot_active, self.running_states['robot'])
        self.update_overview_indicator(self.ind_cam, "Camera:", self.ros_node.camera_active, self.running_states['camera'])
        self.update_overview_indicator(self.ind_seq, "Sequencer:", False, self.running_states['sequencer']) # Sequencer is an active node process, we check running_states mainly
        self.update_overview_indicator(self.ind_ard, "Arduino:", self.ros_node.arduino_active, self.running_states['arduino'])
        self.update_overview_indicator(self.ind_he, "Handeye:", self.ros_node.handeye_active, self.running_states['handeye'])

        # Check Perfect Timing Flags
        if self.ros_node.perfect_timing_robot_flag:
            self.ros_node.perfect_timing_robot_flag = False
            self.log_message("PERFECT TIMING ROBOT")
            self.flash_indicator(self.ind_timing_robot, "#E91E63") # Pinkish

        if self.ros_node.perfect_timing_human_flag:
            self.ros_node.perfect_timing_human_flag = False
            self.log_message("PERFECT TIMING HUMAN")
            self.flash_indicator(self.ind_timing_human, "#2196F3") # Blueish

        # Update Status Indicators text
        self.update_indicator(self.robot_status, self.ros_node.robot_active, self.running_states['robot'])
        self.update_indicator(self.cam_status, self.ros_node.camera_active, self.running_states['camera'])
        self.update_indicator(self.he_status, self.ros_node.handeye_active, self.running_states['handeye'])
        self.update_indicator(self.arduino_driver_status, self.ros_node.arduino_active, self.running_states['arduino'])
        if self.running_states['recording']:
            self.rec_status.setText("RECORDING")
            self.rec_status.setStyleSheet("color: #E91E63; font-weight: bold;")
        else:
            self.rec_status.setText("INACTIVE")
            self.rec_status.setStyleSheet("color: gray; font-weight: bold;")
            
        if self.running_states['playback']:
            self.play_status.setText("PLAYING")
            self.play_status.setStyleSheet("color: #3F51B5; font-weight: bold;")
        else:
            self.play_status.setText("INACTIVE")
            self.play_status.setStyleSheet("color: gray; font-weight: bold;")

        # Update Arduino Monitor Values
        self.val_weight.setText(f"{self.ros_node.current_weight:.2f} g")
        self.val_state.setText(self.ros_node.current_state)
        
        self.txt_led_cmd.setText(self.ros_node.current_led_state)
        led_cmd_color = "gray"
        if self.ros_node.current_led_state == "BLUE": led_cmd_color = "blue"
        elif self.ros_node.current_led_state == "RED": led_cmd_color = "red"
        elif self.ros_node.current_led_state == "GREEN": led_cmd_color = "green"
        self.val_led_cmd.setStyleSheet(f"font-size: 24px; color: {led_cmd_color};")
        
        self.txt_led_stat.setText(self.ros_node.current_led_status)
        led_stat_color = "gray"
        if self.ros_node.current_led_status == "BLUE": led_stat_color = "blue"
        elif self.ros_node.current_led_status == "RED": led_stat_color = "red"
        elif self.ros_node.current_led_status == "GREEN": led_stat_color = "green"
        self.val_led_stat.setStyleSheet(f"font-size: 24px; color: {led_stat_color};")
        
        # Update Safety
        mode = self.ros_node.current_safety_mode
        self.safety_label.setText(mode)
        if self.ros_node.soft_estop_active:
             self.safety_label.setText("SOFT ESTOP")
             self.safety_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        elif "EMERGENCY" in mode or "STOP" in mode or "FAULT" in mode or "VIOLATION" in mode:
             self.safety_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        elif "NORMAL" in mode:
             self.safety_label.setStyleSheet("color: green; font-weight: bold;")
        else:
             self.safety_label.setStyleSheet("color: gray; font-weight: bold;")
        
        # Update Plot
        if len(self.ros_node.times) > 0:
            self.line.set_data(list(self.ros_node.times), list(self.ros_node.weights))
            self.ax.relim()
            self.ax.autoscale_view()
            if self.ros_node.times[-1] > 10:
                self.ax.set_xlim(self.ros_node.times[-1] - 10, self.ros_node.times[-1] + 0.5)
            self.canvas.draw()
        
        # Series auto-advance: check if sequencer finished and series is running
        if self.series_running and self.ros_node.sequence_status == "COMPLETED":
            self.ros_node.sequence_status = "IDLE"  # consume the status
            self._series_finished()

    def flash_indicator(self, label, color):
        label.setStyleSheet(f"background-color: {color}; color: white; padding: 10px; border-radius: 5px; font-weight: bold;")
        QTimer.singleShot(200, lambda: label.setStyleSheet("background-color: lightgray; color: black; padding: 10px; border-radius: 5px; font-weight: bold;"))

    def update_indicator(self, label, active, starting):
        if active:
            label.setText("ACTIVE")
            label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        elif starting:
            label.setText("STARTING")
            label.setStyleSheet("color: #FFC107; font-weight: bold;")
        else:
            label.setText("INACTIVE")
            label.setStyleSheet("color: gray; font-weight: bold;")

    def update_overview_indicator(self, label, prefix, active, starting):
        if active:
            label.setText(f"{prefix} ACTIVE")
            label.setStyleSheet("background-color: #4CAF50; color: white; padding: 5px; border-radius: 5px; font-weight: bold;")
        elif starting:
            label.setText(f"{prefix} STARTING")
            label.setStyleSheet("background-color: #FFC107; color: black; padding: 5px; border-radius: 5px; font-weight: bold;")
        else:
            label.setText(f"{prefix} INACTIVE")
            label.setStyleSheet("background-color: lightgray; color: black; padding: 5px; border-radius: 5px;")

    def closeEvent(self, event):
        # Stop all running processes
        for name, running in self.running_states.items():
            if running:
                self.launch_manager.stop(name)
        event.accept()

def main():
    rclpy.init()
    ros_node = RosNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    launch_manager = LaunchManager()
    window = MainWindow(ros_node, launch_manager)
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
