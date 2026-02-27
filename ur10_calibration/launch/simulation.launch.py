from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10",
            description="Type/series of used UR robot.",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    
    # Custom Description paths
    description_package = "ur10_custom_description"
    description_file = "ur10_with_fist.urdf.xacro"

    # 1. Launch Fake Robot + Controllers (ur10_custom)
    fake_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur10_custom_description"), "launch", "ur10_custom.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "use_fake_hardware": "true",
            "fake_sensor_commands": "true",
            "robot_ip": "192.168.1.102", # Dummy IP
        }.items(),
    )

    # 2. Launch MoveIt (ur_moveit_config)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "true",
            "use_fake_hardware": "true",
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": "ur_moveit_config",
        }.items(),
    )

    # Return
    return LaunchDescription([
        declared_arguments[0], # Add the argument declaration
        fake_robot_launch,
        moveit_launch
    ])
