from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_calibration',
            executable='calibration_gui',
            name='calibration_gui',
            output='screen'
        ),
    ])
