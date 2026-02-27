#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import serial
import time
import threading

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('fake_hardware', False)
        
        self.fake_hardware = self.get_parameter('fake_hardware').value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Fake hardware state
        self.fake_state = "IDLE"
        self.fake_weight = 0.0
        self.fake_led_status = "OFF"
        self.fake_sequence_active = False
        self.fake_weight_offset = 0.0
        
        if self.fake_hardware:
            self.get_logger().info('Running in FAKE HARDWARE mode - no Arduino connection')
            self.serial_conn = None
        else:
            self.get_logger().info(f'Connecting to Arduino on {port} at {baudrate} baud...')
            
            try:
                self.serial_conn = serial.Serial(port, baudrate, timeout=1)
                time.sleep(4) # Wait for Arduino to reset
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.get_logger().info('Connected to Arduino.')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to connect to Arduino: {e}')
                self.destroy_node()
                return

        # Publishers
        self.weight_pub = self.create_publisher(Float32, '/arduino/weight', 10)
        self.led_status_pub = self.create_publisher(String, '/arduino/led_color_status', 10)
        self.led_value_pub = self.create_publisher(Int32, '/arduino/led_value', 10)
        
        # State topic (Transient Local so late subscribers get the last state)
        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/arduino/state', latched_qos)
        
        # Services
        self.srv_stop = self.create_service(Trigger, '/arduino/stop', self.stop_callback)
        self.srv_tare = self.create_service(Trigger, '/arduino/tare', self.tare_callback)
        self.srv_tare = self.create_service(Trigger, '/arduino/tare', self.tare_callback)
        
        # Subscriptions
        self.led_state_sub = self.create_subscription(String, '/arduino/led_color_cmd', self.led_state_callback, 10)
            
        # Serial Reader Thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_loop)
        self.read_thread.start()

    def led_state_callback(self, msg):
        import random
        color = msg.data.upper()
        if color == "RANDOM":
            color = random.choice(["RED", "BLUE"])
            
        if color == "RED":
            self.send_cmd('r')
        elif color == "BLUE":
            self.send_cmd('b')
        elif color == "OFF":
            self.send_cmd('o')
        else:
            self.get_logger().warn(f"Unknown LED state command: {msg.data}")

    def send_cmd(self, cmd):
        if self.fake_hardware:
            # Simulate command execution in fake mode
            self.get_logger().info(f"Fake hardware received command: {cmd}")
            
            if cmd == 's':
                self.fake_state = "IDLE"
                self.fake_sequence_active = False
                self.fake_led_status = "OFF"
            elif cmd == 'r':
                self.fake_led_status = "RED"
            elif cmd == 'b':
                self.fake_led_status = "BLUE"
            elif cmd == 'o':
                self.fake_led_status = "OFF"
            elif cmd == 't':
                self.fake_weight_offset = self.fake_weight
                
            return True, "Fake command executed"
        else:
            try:
                # Ensure the command is encoded and terminated properly if needed (char doesn't need \n in this sketch logic but good practice)
                # The sketch reads char by char: cmd = Serial.read();
                self.serial_conn.write(cmd.encode('utf-8'))
                self.serial_conn.flush()
                return True, "Command sent"
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")
                return False, str(e)

    def stop_callback(self, request, response):
        self.get_logger().info('Service called: Stop')
        success, msg = self.send_cmd('s')
        response.success = success
        response.message = msg
        return response

    def tare_callback(self, request, response):
        self.get_logger().info('Service called: Tare')
        success, msg = self.send_cmd('t')
        response.success = success
        response.message = msg
        return response

    def read_serial_loop(self):
        import random
        import math
        
        loop_count = 0
        while self.running and rclpy.ok():
            if self.fake_hardware:
                # Simulate Arduino data generation
                loop_count += 1
                
                # Publish weight data every ~100ms (10Hz)
                if loop_count % 10 == 0:
                    # Simulate weight with some noise and variation based on sequence
                    base_weight = 100.0 if self.fake_sequence_active else 50.0
                    noise = random.uniform(-2.0, 2.0)
                    # Add a slow sine wave for more realistic variation
                    variation = 5.0 * math.sin(loop_count * 0.01)
                    self.fake_weight = base_weight + noise + variation - self.fake_weight_offset
                    
                    msg = Float32()
                    msg.data = self.fake_weight
                    self.weight_pub.publish(msg)
                
                # Publish LED status every ~500ms
                if loop_count % 50 == 0:
                    msg = String()
                    msg.data = self.fake_led_status
                    self.led_status_pub.publish(msg)
                    
                    # Publish numeric value for plotting
                    val_msg = Int32()
                    if self.fake_led_status == "BLUE":
                        val_msg.data = 1
                    elif self.fake_led_status == "RED":
                        val_msg.data = 2
                    else:
                        val_msg.data = 0
                    self.led_value_pub.publish(val_msg)
                
                # Publish state every ~1s
                if loop_count % 100 == 0:
                    msg = String()
                    msg.data = self.fake_state
                    self.state_pub.publish(msg)
                
                time.sleep(0.01)
                
            elif self.serial_conn.in_waiting > 0:
                try:
                    # Use errors='replace' to ignore bad bytes (e.g. at startup)
                    line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    if line.startswith("DATA:"):
                        # Format: DATA:<time>,<weight>
                        parts = line.split(':')[1].split(',')
                        if len(parts) == 2:
                            weight = float(parts[1])
                            msg = Float32()
                            msg.data = weight
                            self.weight_pub.publish(msg)
                    elif line.startswith("LED:"):
                        # Format: LED:<STATUS>
                        status = line.split(':')[1]
                        msg = String()
                        msg.data = status
                        self.led_status_pub.publish(msg)
                        
                        # Publish numeric value for plotting
                        # OFF=0, BLUE=1, RED=2
                        val_msg = Int32()
                        if status == "BLUE":
                            val_msg.data = 1
                        elif status == "RED":
                            val_msg.data = 2
                        else:
                            val_msg.data = 0
                        self.led_value_pub.publish(val_msg)
                    elif line.startswith("STATE:"):
                        # Format: STATE:<STATUS>
                        status = line.split(':')[1]
                        msg = String()
                        msg.data = status
                        self.state_pub.publish(msg)
                except Exception as e:
                    self.get_logger().warn(f'Error reading serial: {e}')
            else:
                time.sleep(0.01)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join()
        if hasattr(self, 'serial_conn') and self.serial_conn is not None:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = ArduinoBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
