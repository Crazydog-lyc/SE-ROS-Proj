# ========================================================================
# 文件: src/semantic_costmap_plugins/launch/semantic_navigation.launch.py
# 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
# ========================================================================
#
# 【AI-PROMPT】
# 基于 Nav2 Humble costmap_2d::Layer，帮我新建 semantic_costmap_plugins 包骨架：SemanticZoneLayer /
# PreferredLaneLayer / DynamicCongestionLayer 三个插件类，继承 CostmapLayer，先实现
# onInitialize、updateBounds、updateCosts 空壳和 pluginlib 导出，附带 geometry_utils、cost_functions
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    run_headless = LaunchConfiguration("run_headless")
    world_file = LaunchConfiguration("world_file")

    wrapped_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sam_bot_nav2_gz"), "launch", "complete_navigation.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "run_headless": run_headless,
            "world_file": world_file,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("semantic_costmap_plugins"), "config", "nav2_params_semantic.yaml"]
                ),
                description="Full path to the ROS2 parameters file used by Nav2.",
            ),
            DeclareLaunchArgument(
                "run_headless",
                default_value="False",
                description="Start Gazebo without GUI and do not start RViz.",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="World file name relative to the sam_bot_nav2_gz world folder.",
            ),
            wrapped_launch,
        ]
    )
