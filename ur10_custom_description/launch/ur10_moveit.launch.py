from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10",
            description="Type/series of used UR robot.",
        )
    )

    # 1. Launch the robot with fake hardware (Controllers, Robot State Publisher)
    ur_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur10_custom_description"), "launch", "ur10_custom.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_fake_hardware": "true",
            "launch_rviz": "false", # Use MoveIt's RViz instead
        }.items(),
    )

    # 2. Launch MoveIt (Move Group, RViz)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "description_package": "ur10_custom_description",
            "description_file": "ur10_with_fist.urdf.xacro",
            "moveit_config_package": "ur_moveit_config",
            "moveit_config_file": "ur.srdf.xacro",
            "use_sim_time": "false",
            "launch_rviz": "true",
            "launch_servo": "false",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [ur_robot_launch, ur_moveit_launch])
