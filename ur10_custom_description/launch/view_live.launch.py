from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_package = "ur10_custom_description"

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "view_live.rviz"]
    )
    
    # Just launch RViz. It will subscribe to /robot_description and other topics from the live system.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([rviz_node])
