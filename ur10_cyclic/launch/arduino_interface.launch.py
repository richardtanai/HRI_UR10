from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Baudrate for serial connection'
    )
    
    fake_hardware_arg = DeclareLaunchArgument(
        'fake_hardware',
        default_value='false',
        description='Run in fake hardware mode (no Arduino required)'
    )

    arduino_node = Node(
        package='ur10_cyclic',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'fake_hardware': LaunchConfiguration('fake_hardware')
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        fake_hardware_arg,
        arduino_node
    ])

