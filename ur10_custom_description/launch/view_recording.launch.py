from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur10_custom_description",
            description="Description package with robot URDF/Xacro files. Usually the argument is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur10_with_fist.urdf.xacro",
            description="URDF/Xacro description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur10",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
        ]
    )
    
    # Wrap in ParameterValue to avoid YAML parsing error
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "view_robot.rviz"]
    )
    
    # Launch Robot State Publisher (to publish /robot_description and TF)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': True}],
    )
    
    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(declared_arguments + [rsp_node, rviz_node])
