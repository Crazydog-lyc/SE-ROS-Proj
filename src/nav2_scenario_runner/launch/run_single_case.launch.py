from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_file",
                description="Path to generated *_scenario.yaml file.",
            ),
            DeclareLaunchArgument("run_headless", default_value="True"),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            DeclareLaunchArgument("max_retry_per_waypoint", default_value="1"),
            DeclareLaunchArgument("allow_skip_waypoint", default_value="false"),
            DeclareLaunchArgument("nav2_ready_timeout_sec", default_value="420.0"),
            DeclareLaunchArgument("server_timeout_sec", default_value="420.0"),
            DeclareLaunchArgument("auto_start_delay_sec", default_value="120.0"),
            DeclareLaunchArgument("nav2_startup_delay_sec", default_value="50.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("course_bringup"),
                            "launch",
                            "scenario_case.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "scenario_file": LaunchConfiguration("scenario_file"),
                    "run_headless": LaunchConfiguration("run_headless"),
                    "enable_safety": LaunchConfiguration("enable_safety"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "safety_params_file": LaunchConfiguration("safety_params_file"),
                    "max_retry_per_waypoint": LaunchConfiguration(
                        "max_retry_per_waypoint"
                    ),
                    "allow_skip_waypoint": LaunchConfiguration("allow_skip_waypoint"),
                    "nav2_ready_timeout_sec": LaunchConfiguration(
                        "nav2_ready_timeout_sec"
                    ),
                    "server_timeout_sec": LaunchConfiguration("server_timeout_sec"),
                    "auto_start_delay_sec": LaunchConfiguration("auto_start_delay_sec"),
                    "nav2_startup_delay_sec": LaunchConfiguration(
                        "nav2_startup_delay_sec"
                    ),
                }.items(),
            ),
        ]
    )
