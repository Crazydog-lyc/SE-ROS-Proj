# scenario_case.launch.py — 单 case：full_stack + 延迟 auto_run_mission
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import yaml


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


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


def _resolve_start_pose(scenario_file: Path) -> dict:
    try:
        payload = yaml.safe_load(scenario_file.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        payload = {}

    start_pose = payload.get("start_pose") or {}
    return {
        "x": str(float(start_pose.get("x", -2.0))),
        "y": str(float(start_pose.get("y", 0.0))),
        "yaw": str(float(start_pose.get("yaw", 0.0))),
    }


def _build_actions(context):
    scenario_file = Path(LaunchConfiguration("scenario_file").perform(context)).resolve()
    base_name = scenario_file.name.replace("_scenario.yaml", "")
    mission_file = scenario_file.with_name(f"{base_name}_mission.json")
    semantic_overlay = scenario_file.with_name(f"{base_name}_semantic_overlay.yaml")

    requested_world_file = LaunchConfiguration("world_file").perform(context).strip()
    world_file = requested_world_file or _resolve_world_file(scenario_file)
    start_pose = _resolve_start_pose(scenario_file)
    spawn_from_scenario = _as_bool(
        LaunchConfiguration("spawn_from_scenario").perform(context)
    )
    spawn_pose = start_pose if spawn_from_scenario else {
        "x": LaunchConfiguration("spawn_x").perform(context),
        "y": LaunchConfiguration("spawn_y").perform(context),
        "yaw": LaunchConfiguration("spawn_yaw").perform(context),
    }
    requested_nav2 = LaunchConfiguration("nav2_params_file").perform(context).strip()
    nav2_params_file = requested_nav2 or (
        str(semantic_overlay) if semantic_overlay.exists() else ""
    )

    nav2_ready_timeout = LaunchConfiguration("nav2_ready_timeout_sec").perform(context)
    server_timeout = LaunchConfiguration("server_timeout_sec").perform(context)
    auto_start_delay = float(LaunchConfiguration("auto_start_delay_sec").perform(context))

    bringup_share = Path(get_package_share_directory("course_bringup"))
    full_stack_launch = bringup_share / "launch" / "full_stack.launch.py"

    auto_runner = Node(
        package="course_bringup",
        executable="auto_run_mission.py",
        name=f"{base_name}_auto_run_mission",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "mission_file": str(mission_file),
                "max_retry_per_waypoint": LaunchConfiguration("max_retry_per_waypoint"),
                "allow_skip_waypoint": LaunchConfiguration("allow_skip_waypoint"),
                "nav2_ready_timeout_sec": float(nav2_ready_timeout),
                "server_timeout_sec": float(server_timeout),
            }
        ],
    )

    actions = [
        LogInfo(msg=f"Launching scenario case from {scenario_file}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(full_stack_launch)),
            launch_arguments={
                "run_headless": LaunchConfiguration("run_headless"),
                "world_file": world_file,
                "spawn_x": spawn_pose["x"],
                "spawn_y": spawn_pose["y"],
                "spawn_yaw": spawn_pose["yaw"],
                "nav2_params_file": nav2_params_file,
                "mission_file": str(mission_file),
                "enable_semantic": "True" if semantic_overlay.exists() else "False",
                "enable_safety": LaunchConfiguration("enable_safety"),
                "safety_params_file": LaunchConfiguration("safety_params_file"),
                "start_mission_manager": "True",
                "nav2_startup_delay_sec": LaunchConfiguration("nav2_startup_delay_sec"),
            }.items(),
        ),
        TimerAction(period=auto_start_delay, actions=[auto_runner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=auto_runner,
                on_exit=[Shutdown(reason=f"Scenario {base_name} completed")],
            )
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file"),
            DeclareLaunchArgument("run_headless", default_value="True"),
            DeclareLaunchArgument("world_file", default_value=""),
            DeclareLaunchArgument(
                "spawn_from_scenario",
                default_value="False",
                description="Use scenario start_pose as Gazebo spawn. Default keeps the robot at the known-safe simulator spawn.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="-2.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            DeclareLaunchArgument("max_retry_per_waypoint", default_value="1"),
            DeclareLaunchArgument("allow_skip_waypoint", default_value="false"),
            DeclareLaunchArgument(
                "nav2_ready_timeout_sec",
                default_value="420.0",
                description="Seconds to wait for Nav2 lifecycle nodes before sending mission.",
            ),
            DeclareLaunchArgument(
                "server_timeout_sec",
                default_value="420.0",
                description="Seconds to wait for /mission/run action server.",
            ),
            DeclareLaunchArgument(
                "auto_start_delay_sec",
                default_value="120.0",
                description="Delay before auto_run_mission starts (stack boot headroom).",
            ),
            DeclareLaunchArgument(
                "nav2_startup_delay_sec",
                default_value="50.0",
                description="Passed to complete_navigation: SLAM -> Nav2 delay.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
