from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Optional: If you want to change distance from command line
    # Usage: ros2 launch ur10_cyclic safety_system.launch.py safety_dist:=1.0
    dist_arg = DeclareLaunchArgument(
        'safety_dist', default_value='0.75',
        description='Distance in meters to trigger stop'
    )

    # 1. The Safety Node
    safety_node = Node(
        package='ur10_cyclic',
        executable='human_safety', # Must match 'entry_points' in setup.py
        name='human_safety_node',
        output='screen',
        parameters=[
            {'safety_distance': LaunchConfiguration('safety_dist')}
        ],
        # --- REMAPPINGS ---
        # "Node expects this (Left)"  ->  "System provides this (Right)"
        remappings=[
            ('/camera/color/image_raw',               '/camera/camera/color/image_raw'),
            ('/camera/aligned_depth_to_color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/color/camera_info',             '/camera/camera/color/camera_info'),
            # Connect the safety output to your robot controller topic
            ('/safety/emergency_stop',                '/ur10/safety_stop_signal') 
        ]
    )

    return LaunchDescription([
        dist_arg,
        safety_node
    ])