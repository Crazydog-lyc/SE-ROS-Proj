from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("nav2_mission_manager"), "config", "mission_manager.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Optional parameter file for nav2_mission_manager.",
            ),
            Node(
                package="nav2_mission_manager",
                executable="mission_manager_node",
                name="mission_action_server",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
