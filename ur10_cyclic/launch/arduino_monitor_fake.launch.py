from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Include the Arduino Interface launch file with fake_hardware set to true
    arduino_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur10_cyclic'),
                'launch',
                'arduino_interface.launch.py'
            ])
        ]),
        launch_arguments={
            'fake_hardware': 'true',
            'port': '/dev/ttyFAKE',  # Dummy port
        }.items()
    )

    # Launch the RQT Monitor GUI
    rqt_monitor_node = Node(
        package='ur10_cyclic',
        executable='rqt_arduino_monitor.py',
        name='rqt_arduino_monitor',
        output='screen'
    )

    return LaunchDescription([
        arduino_interface_launch,
        rqt_monitor_node
    ])
