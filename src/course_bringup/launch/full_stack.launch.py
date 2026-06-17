# ========================================================================
# 文件: src/course_bringup/launch/full_stack.launch.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# course_bringup/full_stack.launch.py：Include sam_bot_nav2_gz complete_navigation，可选启动
# mission_manager、safety_monitor，nav2_params 在 base/semantic 间切换。请用 OpaqueFunction +
# DeclareLaunchArgument 生成 launch 骨架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# 默认 map/params 在 course_bringup/config，可用 launch arg 覆盖
# ========================================================================
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # 组合子 launch，见各 IncludeLaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration  # 参数来自 launch arg
from launch_ros.actions import Node
from pathlib import Path


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _build_actions(context):
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context).strip()
    enable_semantic = _as_bool(LaunchConfiguration("enable_semantic").perform(context))
    start_mission_manager = _as_bool(
        LaunchConfiguration("start_mission_manager").perform(context)
    )

    base_share = Path(get_package_share_directory("sam_bot_nav2_gz"))
    semantic_share = Path(get_package_share_directory("semantic_costmap_plugins"))
    if not nav2_params_file:
        nav2_params_file = str(
            semantic_share / "config" / "nav2_params_semantic.yaml"
            if enable_semantic
            else base_share / "config" / "nav2_params.yaml"
        )

    safety_params_file = LaunchConfiguration("safety_params_file").perform(context).strip()
    if not safety_params_file:
        safety_params_file = str(base_share / "config" / "safety_monitor.yaml")

    base_launch = PythonLaunchDescriptionSource(
        str(base_share / "launch" / "complete_navigation.launch.py")
    )

    actions = [
        IncludeLaunchDescription(
            base_launch,
            launch_arguments={
                "params_file": nav2_params_file,
                "run_headless": LaunchConfiguration("run_headless"),
                "world_file": LaunchConfiguration("world_file"),
            }.items(),
        )
    ]
    actions.append(
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("enable_safety")),
            name="safety_monitor",
            cmd=[
                "python3",
                "-m",
                "sam_bot_safety_monitor.safety_monitor",
                "--ros-args",
                "--params-file",
                safety_params_file,
            ],
            output="screen",
        )
    )
    if start_mission_manager:
        actions.append(
            Node(
                package="nav2_mission_manager",
                executable="mission_manager_node",
                name="mission_action_server",
                output="screen",
            )
        )

    return actions



# 一键起 Gazebo + Nav2 + mission + safety
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_headless", default_value="False"),
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            DeclareLaunchArgument("nav2_params_file", default_value=""),
            DeclareLaunchArgument("mission_file", default_value=""),
            DeclareLaunchArgument("enable_semantic", default_value="False"),
            DeclareLaunchArgument("enable_safety", default_value="False"),
            DeclareLaunchArgument("safety_params_file", default_value=""),
            DeclareLaunchArgument("start_mission_manager", default_value="False"),
            OpaqueFunction(function=_build_actions),
        ]
    )
