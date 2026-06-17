# ========================================================================
# 文件: src/course_bringup/launch/mission_demo.launch.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# mission_demo.launch.py 一键启动 Nav2 + mission_manager，run_headless 参数传给 Gazebo，请生成最小 launch
# 描述。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# 默认 map/params 在 course_bringup/config
# ========================================================================
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # 组合子 launch，见各 IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration  # 参数来自 launch arg
from pathlib import Path



# 只起 mission_manager + Nav2，不含 Gazebo
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
