# ========================================================================
# 文件: src/nav2_scenario_runner/launch/run_single_case.launch.py
# 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
# ========================================================================
#
# 【AI-PROMPT】
# 帮我 scaffold 一个 nav2_scenario_runner C++ 包：pluginlib 注册 ScenarioGenerator 插件，包含
# generator_registry、scenario_types、scenario_serializer，再写 generate_scenario_node 和 Python
# 侧 generate_cases/run_batch 脚本入口。要求固定 random seed、每个 case 输出独立目录；具体 corridor/room
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
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
                }.items(),
            ),
        ]
    )
