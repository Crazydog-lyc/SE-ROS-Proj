import os
import sys

import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO, OnProcessStart

sys.path.insert(0, os.path.dirname(__file__))
from nav2_params_utils import patch_nav2_params  # noqa: E402


def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


def _launch_setup(context, *args, **kwargs):
    diff_drive_loaded_message = (
        "Successfully loaded controller diff_drive_base_controller into state active"
    )
    navigation_ready_message = "Creating bond timer"

    run_headless = LaunchConfiguration("run_headless")
    world_file = LaunchConfiguration("world_file")
    nav2_startup_delay_sec = LaunchConfiguration("nav2_startup_delay_sec")
    params_file = LaunchConfiguration("params_file").perform(context)
    patched_params_file = patch_nav2_params(params_file)

    bringup = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("sam_bot_nav2_gz"),
                    "launch",
                    "display.launch.py",
                ]
            ),
            "use_rviz:=false",
            ["run_headless:=", run_headless],
            "use_localization:=true",
            ["world_file:=", world_file],
            ["spawn_x:=", LaunchConfiguration("spawn_x")],
            ["spawn_y:=", LaunchConfiguration("spawn_y")],
            ["spawn_z:=", LaunchConfiguration("spawn_z")],
            ["spawn_yaw:=", LaunchConfiguration("spawn_yaw")],
        ],
        shell=False,
        output="screen",
    )
    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            ),
        ],
        shell=False,
        output="screen",
    )
    waiting_toolbox = RegisterEventHandler(
        OnProcessIO(
            target_action=bringup,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(
                        msg="Diff drive controller loaded. Starting SLAM Toolbox..."
                    ),
                    toolbox,
                ],
            ),
        )
    )

    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
            "use_sim_time:=True",
            f"params_file:={patched_params_file}",
        ],
        shell=False,
        output="screen",
    )
    rviz_node = Node(
        condition=IfCondition(NotSubstitution(run_headless)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )
    waiting_navigation = RegisterEventHandler(
        OnProcessStart(
            target_action=toolbox,
            on_start=[
                LogInfo(msg="SLAM Toolbox started; scheduling Nav2 launch..."),
                TimerAction(
                    period=nav2_startup_delay_sec,
                    actions=[navigation, rviz_node],
                ),
            ],
        )
    )
    waiting_success = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [
                    LogInfo(msg="Ready for navigation!"),
                ],
            ),
        )
    )

    return [bringup, waiting_toolbox, waiting_navigation, waiting_success]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=[
                    FindPackageShare("sam_bot_nav2_gz"),
                    "/config/nav2_params.yaml",
                ],
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("sam_bot_nav2_gz"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            DeclareLaunchArgument(name="spawn_x", default_value="-2.0"),
            DeclareLaunchArgument(name="spawn_y", default_value="0.0"),
            DeclareLaunchArgument(name="spawn_z", default_value="1.0"),
            DeclareLaunchArgument(name="spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                name="nav2_startup_delay_sec",
                default_value="50.0",
                description="Seconds after SLAM toolbox starts before launching Nav2.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
