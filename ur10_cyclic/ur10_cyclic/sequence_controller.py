#!/usr/bin/env python3
import sys
import threading
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QGroupBox
from PyQt5.QtCore import QTimer, Qt

class SequenceController(Node):
    def __init__(self):
        super().__init__('sequence_controller')
        
        # --- Robot Sequence Clients ---
        self.cli_next = self.create_client(Trigger, '/aruco_sequence/next_pose')
        self.cli_reset = self.create_client(Trigger, '/aruco_sequence/reset')
        
        # --- Arduino Control Clients ---
        self.cli_start_1 = self.create_client(Trigger, '/arduino/start_sequence_1')
        self.cli_start_3 = self.create_client(Trigger, '/arduino/start_sequence_3')
        self.cli_stop = self.create_client(Trigger, '/arduino/stop')
        self.cli_tare = self.create_client(Trigger, '/arduino/tare')
        
        self.latest_status = "Ready"

    def send_request(self, client, description):
        if not client.service_is_ready():
            self.latest_status = f"Service '{description}' not available!"
            return

        self.latest_status = f"Sending '{description}'..."
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.latest_status = f"{response.message}" if response.message else "Success"
        except Exception as e:
            self.latest_status = f"Service call failed: {e}"

class Gui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(100)

    def initUI(self):
        main_layout = QVBoxLayout()
        
        # --- Robot Control Group ---
        robot_group = QGroupBox("Robot ArUco Sequence")
        robot_layout = QVBoxLayout()
        
        btn_next = QPushButton('Next Pose')
        btn_next.setStyleSheet("font-size: 16px; padding: 10px; background-color: #4CAF50; color: white;")
        btn_next.clicked.connect(lambda: self.node.send_request(self.node.cli_next, "Next Pose"))
        robot_layout.addWidget(btn_next)

        btn_reset = QPushButton('Reset Sequence')
        btn_reset.setStyleSheet("font-size: 14px; padding: 5px; background-color: #ff9800; color: white;")
        btn_reset.clicked.connect(lambda: self.node.send_request(self.node.cli_reset, "Reset Sequence"))
        robot_layout.addWidget(btn_reset)
        
        robot_group.setLayout(robot_layout)
        main_layout.addWidget(robot_group)
        
        # --- Arduino Control Group ---
        arduino_group = QGroupBox("Arduino Highbay Control")
        arduino_layout = QVBoxLayout()
        
        # Row 1: Sequences
        h_layout1 = QHBoxLayout()
        btn_seq1 = QPushButton('Seq 1 (Var 1.1)')
        btn_seq1.setStyleSheet("padding: 8px; background-color: #2196F3; color: white;")
        btn_seq1.clicked.connect(lambda: self.node.send_request(self.node.cli_start_1, "Seq 1"))
        
        btn_seq3 = QPushButton('Seq 3 (Var 3.3)')
        btn_seq3.setStyleSheet("padding: 8px; background-color: #2196F3; color: white;")
        btn_seq3.clicked.connect(lambda: self.node.send_request(self.node.cli_start_3, "Seq 3"))
        
        h_layout1.addWidget(btn_seq1)
        h_layout1.addWidget(btn_seq3)
        arduino_layout.addLayout(h_layout1)
        
        # Row 2: Stop & Tare
        h_layout2 = QHBoxLayout()
        btn_stop = QPushButton('STOP')
        btn_stop.setStyleSheet("padding: 8px; background-color: #f44336; color: white; font-weight: bold;")
        btn_stop.clicked.connect(lambda: self.node.send_request(self.node.cli_stop, "STOP"))
        
        btn_tare = QPushButton('Tare Scale')
        btn_tare.setStyleSheet("padding: 8px; background-color: #9C27B0; color: white;")
        btn_tare.clicked.connect(lambda: self.node.send_request(self.node.cli_tare, "Tare"))
        
        h_layout2.addWidget(btn_stop)
        h_layout2.addWidget(btn_tare)
        arduino_layout.addLayout(h_layout2)
        
        arduino_group.setLayout(arduino_layout)
        main_layout.addWidget(arduino_group)

        # --- Status Label ---
        self.label = QLabel("Ready")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 12px; margin: 5px; color: #555;")
        main_layout.addWidget(self.label)

        self.setLayout(main_layout)
        self.setWindowTitle('UR10 & Arduino Controller')
        self.setGeometry(100, 100, 350, 400)

    def update_status(self):
        self.label.setText(self.node.latest_status)

def main(args=None):
    rclpy.init(args=args)
    node = SequenceController()
    
    # Spin ROS in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    app = QApplication(sys.argv)
    gui = Gui(node)
    gui.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
