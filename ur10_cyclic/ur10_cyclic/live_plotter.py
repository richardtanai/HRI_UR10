#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        
        # Data buffers (max 200 points)
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        self.weights = deque(maxlen=self.max_points)
        self.led_values = deque(maxlen=self.max_points)
        
        self.start_time = None

        self.create_subscription(Float32, '/arduino/weight', self.weight_callback, 10)
        self.create_subscription(Int32, '/arduino/led_value', self.led_callback, 10)
        
        self.current_weight = 0.0
        self.current_led = 0

    def weight_callback(self, msg):
        self.current_weight = msg.data
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

    def led_callback(self, msg):
        self.current_led = msg.data

    def update_data(self):
        if self.start_time is None:
            return

        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        self.times.append(current_time)
        self.weights.append(self.current_weight)
        self.led_values.append(self.current_led)

def animate(frame, node, lines, axes):
    node.update_data()
    
    if not node.times:
        return lines

    # Update Weight Plot
    lines[0].set_data(node.times, node.weights)
    
    # Update LED Plot
    lines[1].set_data(node.times, node.led_values)
    
    # Adjust axes
    for ax in axes:
        ax.relim()
        ax.autoscale_view()
        ax.set_xlim(left=max(0, node.times[-1] - 10), right=node.times[-1] + 1)
        
    return lines

    return lines

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()
    
    # Spin ROS in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Matplotlib Setup
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    
    # Weight Plot
    line1, = ax1.plot([], [], 'b-', label='Weight (g)')
    ax1.set_ylabel('Weight')
    ax1.legend()
    ax1.grid(True)
    
    # LED Status Plot
    line2, = ax2.plot([], [], 'r-', label='LED Status')
    ax2.set_ylabel('LED State')
    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels(['OFF', 'BLUE', 'RED'])
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')

    ani = animation.FuncAnimation(fig, animate, fargs=(node, [line1, line2], [ax1, ax2]), interval=100)
    
    plt.show()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
