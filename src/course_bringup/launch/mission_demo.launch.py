from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    share_dir = Path(get_package_share_directory("course_bringup"))
    full_stack = share_dir / "launch" / "full_stack.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_headless", default_value="False"),
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument(
                "mission_file",
                default_value=str(share_dir / "config" / "sample_mission.json"),
            ),
            DeclareLaunchArgument("enable_semantic", default_value="False"),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(full_stack)),
                launch_arguments={
                    "run_headless": LaunchConfiguration("run_headless"),
                    "world_file": LaunchConfiguration("world_file"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "mission_file": LaunchConfiguration("mission_file"),
                    "enable_semantic": LaunchConfiguration("enable_semantic"),
                    "enable_safety": LaunchConfiguration("enable_safety"),
                    "safety_params_file": LaunchConfiguration("safety_params_file"),
                    "start_mission_manager": "True",
                }.items(),
            ),
        ]
    )
