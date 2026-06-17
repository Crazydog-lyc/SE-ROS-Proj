# ========================================================================
# 文件: src/nav2_mission_manager/launch/mission_manager.launch.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 我在 ROS2 Humble 上做 Nav2 课程项目，需要新建 nav2_mission_manager 包。请帮我搭 Python Action Server
# 骨架：/mission/run 用 course_interfaces/RunMission，ReentrantCallbackGroup +
# MultiThreadedExecutor，发布 /mission/state，订阅 /safety/state；先把
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
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
