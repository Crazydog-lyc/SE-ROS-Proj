from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import yaml


def _resolve_world_file(scenario_file: Path) -> str:
    try:
        payload = yaml.safe_load(scenario_file.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        return "empty.sdf"

    metadata = payload.get("metadata") or {}
    world_file = metadata.get("world_file")
    if isinstance(world_file, str) and world_file.strip():
        return world_file
    return "empty.sdf"


def _build_actions(context):
    scenario_file = Path(LaunchConfiguration("scenario_file").perform(context)).resolve()
    base_name = scenario_file.name.replace("_scenario.yaml", "")
    mission_file = scenario_file.with_name(f"{base_name}_mission.json")
    semantic_overlay = scenario_file.with_name(f"{base_name}_semantic_overlay.yaml")

    requested_world_file = LaunchConfiguration("world_file").perform(context).strip()
    world_file = requested_world_file or _resolve_world_file(scenario_file)
    requested_nav2 = LaunchConfiguration("nav2_params_file").perform(context).strip()
    nav2_params_file = requested_nav2 or (
        str(semantic_overlay) if semantic_overlay.exists() else ""
    )

    bringup_share = Path(get_package_share_directory("course_bringup"))
    full_stack_launch = bringup_share / "launch" / "full_stack.launch.py"

    actions = [
        LogInfo(msg=f"Launching scenario case from {scenario_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(full_stack_launch)),
            launch_arguments={
                "run_headless": LaunchConfiguration("run_headless"),
                "world_file": world_file,
                "nav2_params_file": nav2_params_file,
                "mission_file": str(mission_file),
                "enable_semantic": "True" if semantic_overlay.exists() else "False",
                "enable_safety": LaunchConfiguration("enable_safety"),
                "safety_params_file": LaunchConfiguration("safety_params_file"),
                "start_mission_manager": "True",
            }.items(),
        ),
    ]

    auto_runner = Node(
        package="course_bringup",
        executable="auto_run_mission.py",
        name=f"{base_name}_auto_run_mission",
        output="screen",
        parameters=[
            {
                "mission_file": str(mission_file),
                "max_retry_per_waypoint": LaunchConfiguration(
                    "max_retry_per_waypoint"
                ),
                "allow_skip_waypoint": LaunchConfiguration("allow_skip_waypoint"),
            }
        ],
    )
    actions.append(auto_runner)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=auto_runner,
                on_exit=[Shutdown(reason=f"Scenario {base_name} completed")],
            )
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file"),
            DeclareLaunchArgument("run_headless", default_value="True"),
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            DeclareLaunchArgument("max_retry_per_waypoint", default_value="1"),
            DeclareLaunchArgument("allow_skip_waypoint", default_value="false"),
            OpaqueFunction(function=_build_actions),
        ]
    )
