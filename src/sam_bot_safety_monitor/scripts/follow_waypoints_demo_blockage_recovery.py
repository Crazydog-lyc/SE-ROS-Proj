# ========================================================================
# 文件: src/sam_bot_safety_monitor/scripts/follow_waypoints_demo_blockage_recovery.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# 多点 followWaypoints Demo，集成 safety_monitor，展示 blockage recovery。请生成 navigator + safety
# 订阅的主循环框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
#! /usr/bin/env python3

# ---------------------------------------------------------------------------
# 【演示脚本说明】Gazebo 联调用；print 便于录屏。需先起 safety_monitor + Nav2。
# ---------------------------------------------------------------------------


import argparse
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.utilities import remove_ros_args

from sam_bot_safety_monitor.safety_navigation import SafetyAwareNavigator


# A simple clockwise ring around the demo room so the motion is easy to read in RViz/Gazebo.
# 和 follow_waypoints_demo 同路线，专门测堵塞 recovery
DEMO_ROUTE = [
    (-1.6, 0.0, 1.05),
    (-1.15, 1.1, 0.6),
    (0.0, 1.55, 0.0),
    (1.15, 1.1, -0.6),
    (1.6, 0.0, -1.05),
    (1.15, -1.1, -2.55),
    (0.0, -1.55, 3.14),
    (-1.15, -1.1, 2.55),
]



# 堵塞场景下 recovery 后是否整圈重跑
def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a safety demo with explicit post-recovery full mission replay."
    )
    parser.add_argument("--restart-anchor-x", type=float, default=-1.6)
    parser.add_argument("--restart-anchor-y", type=float, default=0.0)
    parser.add_argument("--restart-anchor-yaw", type=float, default=1.05)
    parser.add_argument("--navigation-timeout", type=float, default=900.0)
    return parser.parse_args(remove_ros_args(args=None)[1:])


# map 坐标系下的 PoseStamped
def _create_pose(navigator: SafetyAwareNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


# 把 DEMO_ROUTE 转成 Nav2 可用的 pose 列表
def _build_demo_route(navigator: SafetyAwareNavigator):
    return [_create_pose(navigator, x, y, yaw) for x, y, yaw in DEMO_ROUTE]


# recovery 完成后先回到的起始锚点
def _build_restart_anchor_pose(
    navigator: SafetyAwareNavigator, x: float, y: float, yaw: float
) -> PoseStamped:
    return _create_pose(navigator, x, y, yaw)


# 重放前 cancel 残留 Nav2 goal
def _cancel_active_task_if_needed(navigator: SafetyAwareNavigator) -> None:
    if navigator.result_future is not None:
        navigator.cancelTask()


# 第一阶段：goToPose 回锚点
def _start_anchor_return(
    navigator: SafetyAwareNavigator, restart_anchor_pose: PoseStamped
) -> bool:
    print("[DEMO C] returning to restart anchor pose", flush=True)
    return navigator.goToPose(restart_anchor_pose)


# 第二阶段：followWaypoints 整圈重跑
def _start_full_mission_replay(
    navigator: SafetyAwareNavigator, original_goals
) -> bool:
    return navigator.followWaypoints(original_goals)


# 主流程：followWaypoints + 监听 RECOVERING 后重放
def main():
    args = parse_args()
    rclpy.init()
    navigator = SafetyAwareNavigator(
        node_name="follow_waypoints_demo_blockage_recovery_navigator",
        recovery_mode="client_replay_after_normal",
    )

    print("[DEMO C] blockage recovery demonstration", flush=True)
    print("[DEMO C] demo route: 8-waypoint ring loop around the room", flush=True)
    print(f"[DEMO C] recovery mode: {navigator.get_recovery_mode()}", flush=True)

    navigator.waitUntilNav2Active(localizer="smoother_server")
    print("[DEMO C] Nav2 active!", flush=True)

    original_goals = _build_demo_route(navigator)
    restart_anchor_pose = _build_restart_anchor_pose(
        navigator,
        args.restart_anchor_x,
        args.restart_anchor_y,
        args.restart_anchor_yaw,
    )

    print(f"[DEMO C] demo route ready with {len(original_goals)} waypoints", flush=True)
    print(
        "[DEMO C] restart anchor pose: "
        + f"({args.restart_anchor_x:.2f}, {args.restart_anchor_y:.2f}, {args.restart_anchor_yaw:.2f})",
        flush=True,
    )

    nav_start = navigator.get_clock().now()
    if not navigator.followWaypoints(original_goals):
        print(
            "[DEMO C] waypoint mission was rejected because safety monitor is not ready for navigation.",
            flush=True,
        )
        raise SystemExit(1)

    feedback_counter = 0
    was_recovering = False
    replay_after_recovery_enabled = True
    replay_done = False
    replay_in_progress = False
    anchor_return_active = False
    full_mission_replay_active = False
    demo_failed = False
    last_safety_state = None

    # 轮询任务完成与安全态，和 demo 脚本打印对齐
    while rclpy.ok():
        task_complete = navigator.isTaskComplete()
        safety_state = navigator.get_safety_state_label()

        # 安全态变化时打印，方便对照 Gazebo
        if safety_state != last_safety_state:
            print(f"[DEMO C] current safety state: {safety_state}", flush=True)
            if safety_state in ("PAUSED", "EMERGENCY_STOP", "RECOVERING"):
                print(
                    "[DEMO C] auto-detected safety event: "
                    + f"{navigator.get_safety_source()} - {navigator.get_safety_reason()}",
                    flush=True,
                )
            if safety_state == "PAUSED":
                print("[DEMO C] safety state changed to PAUSED", flush=True)
                print("[DEMO C] waiting for recovery...", flush=True)
            elif safety_state == "EMERGENCY_STOP":
                print("[DEMO C] safety state changed to EMERGENCY_STOP", flush=True)
            elif safety_state == "RECOVERING":
                was_recovering = True
                print("[DEMO C] safety state changed to RECOVERING", flush=True)
                print("[DEMO C] waiting for demo replay after recovery", flush=True)
            elif safety_state == "NORMAL":
                if was_recovering and replay_after_recovery_enabled and not replay_done:
                    print("[DEMO C] detected recovery completion", flush=True)
                if last_safety_state == "RECOVERING":
                    print("[DEMO C] navigation resumed", flush=True)
            elif safety_state == "CANCELED":
                print("[DEMO C] exiting because task is canceled", flush=True)
            last_safety_state = safety_state

        if navigator.is_waiting_for_recovery():
            # 驱动 safety 订阅与 Nav2 回调
            rclpy.spin_once(navigator, timeout_sec=0.1)
            continue

        if was_recovering and safety_state == "NORMAL" and replay_after_recovery_enabled and not replay_done:
            was_recovering = False
            replay_done = True
            replay_in_progress = True
            _cancel_active_task_if_needed(navigator)

            if _start_anchor_return(navigator, restart_anchor_pose):
                anchor_return_active = True
                full_mission_replay_active = False
                nav_start = navigator.get_clock().now()
                rclpy.spin_once(navigator, timeout_sec=0.1)
                continue

            print("[DEMO C] anchor return failed, replaying from current pose", flush=True)
            if _start_full_mission_replay(navigator, original_goals):
                anchor_return_active = False
                full_mission_replay_active = True
                nav_start = navigator.get_clock().now()
                print("[DEMO C] full mission replay started", flush=True)
                rclpy.spin_once(navigator, timeout_sec=0.1)
                continue

            print("[DEMO C] full mission replay request was rejected", flush=True)
            demo_failed = True
            break

        # Nav2 报告当前 goal 结束
        if task_complete:
            if anchor_return_active:
                anchor_return_active = False
                if navigator.status == GoalStatus.STATUS_SUCCEEDED:
                    print("[DEMO C] anchor reached, replaying full mission", flush=True)
                else:
                    print("[DEMO C] anchor return failed, replaying from current pose", flush=True)

                if _start_full_mission_replay(navigator, original_goals):
                    full_mission_replay_active = True
                    nav_start = navigator.get_clock().now()
                    print("[DEMO C] full mission replay started", flush=True)
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                    continue

                print("[DEMO C] full mission replay request was rejected", flush=True)
                demo_failed = True
                break

            # 重放阶段完成
            if full_mission_replay_active:
                if navigator.status == GoalStatus.STATUS_SUCCEEDED:
                    print("[DEMO C] full mission replay completed", flush=True)
                full_mission_replay_active = False
                replay_in_progress = False
                break

            if replay_in_progress:
                replay_in_progress = False

            break

        # 周期性打印 Nav2 feedback
        feedback_counter += 1
        feedback = navigator.getFeedback()
        if feedback and feedback_counter % 5 == 0 and not anchor_return_active:
            print(
                "[DEMO C] executing current waypoint: "
                + f"{navigator.get_current_waypoint_index() + 1}/{len(original_goals)}",
                flush=True,
            )
            if navigator.get_clock().now() - nav_start > Duration(
                seconds=args.navigation_timeout
            ):
                print("[DEMO C] navigation canceled / failed", flush=True)
                navigator.cancelTask()

        rclpy.spin_once(navigator, timeout_sec=0.1)

    # 重放失败则退出 demo
    if demo_failed:
        print("[DEMO C] navigation canceled / failed", flush=True)
        raise SystemExit(1)

    result = navigator.getResult()
    # 整圈跑完，demo 成功
    if result == TaskResult.SUCCEEDED:
        print("[DEMO C] goal succeeded!", flush=True)
        if replay_done:
            print("[DEMO C] navigation finished after full mission replay", flush=True)
        else:
            print("[DEMO C] navigation finished at final demo waypoint", flush=True)
    elif result == TaskResult.CANCELED:
        if navigator.is_terminal_canceled():
            print("[DEMO C] goal was canceled!", flush=True)
            print("[DEMO C] navigation canceled / failed", flush=True)
        else:
            print("[DEMO C] goal was canceled without a terminal safety cancel state!", flush=True)
    elif result == TaskResult.FAILED:
        print("[DEMO C] goal failed!", flush=True)
        print("[DEMO C] navigation canceled / failed", flush=True)
    else:
        print("[DEMO C] goal has an invalid return status!", flush=True)

    raise SystemExit(0)


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------------
# 【联调检查清单】
# 录屏时建议同时开 rqt 看 /safety/state 与 /mission/state
# 堵塞 demo 记得开 enable_blockage_monitor
# 1. safety_monitor 已 active 且 /scan /odom 有数据
# 2. Nav2 lifecycle 到 active，localizer 与 launch 一致
# 3. RViz 里能看到 robot 与 costmap
# 4. 故意触发 pause 时 mission_manager 应进入 PAUSED_FOR_SAFETY
# ---------------------------------------------------------------------------
