from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file", description="Generated scenario yaml."),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional params file override.",
            ),
            DeclareLaunchArgument("run_headless", default_value="True"),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            DeclareLaunchArgument("max_retry_per_waypoint", default_value="1"),
            DeclareLaunchArgument("allow_skip_waypoint", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nav2_scenario_runner"),
                            "launch",
                            "run_single_case.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "scenario_file": LaunchConfiguration("scenario_file"),
                    "run_headless": LaunchConfiguration("run_headless"),
                    "enable_safety": LaunchConfiguration("enable_safety"),
                    "nav2_params_file": LaunchConfiguration("params_file"),
                    "safety_params_file": LaunchConfiguration("safety_params_file"),
                    "max_retry_per_waypoint": LaunchConfiguration(
                        "max_retry_per_waypoint"
                    ),
                    "allow_skip_waypoint": LaunchConfiguration("allow_skip_waypoint"),
                }.items(),
            ),
        ]
    )
