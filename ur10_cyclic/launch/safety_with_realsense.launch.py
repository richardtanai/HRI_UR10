from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- 1. Arguments ---
    # Allow changing safety distance from CLI
    dist_arg = DeclareLaunchArgument(
        'safety_dist', default_value='0.75',
        description='Distance in meters to trigger stop'
    )

    # --- 2. RealSense Camera Launch ---
    # We include the standard realsense launch file but force 'align_depth' to true
    realsense_dir = get_package_share_directory('realsense2_camera')
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',  # CRITICAL: Needed for 3D mapping
            'pointcloud.enable': 'false',  # Save CPU
            'camera_name': 'camera'        # This usually creates /camera/camera/...
        }.items()
    )

    # --- 3. The Safety Node ---
    safety_node = Node(
        package='ur10_cyclic',
        executable='human_safety',
        name='human_safety_node',
        output='screen',
        parameters=[
            {'safety_distance': LaunchConfiguration('safety_dist')}
        ],
        remappings=[
            # Remap internal node topics (Left) to actual camera topics (Right)
            # Adjust the right side if your camera does NOT use double namespaces
            ('/camera/color/image_raw',                 '/camera/camera/color/image_raw'), 
            ('/camera/aligned_depth_to_color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/color/camera_info',               '/camera/camera/color/camera_info'),
            ('/safety/emergency_stop',                  '/ur10/safety_stop_signal')
        ]
    )

    # Wait 5 seconds for camera to initialize before starting the AI
    delayed_safety_node = TimerAction(
        period=5.0,
        actions=[safety_node]
    )

    return LaunchDescription([
        dist_arg,
        realsense_launch,
        delayed_safety_node
    ])