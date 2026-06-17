# ========================================================================
# 文件: src/sam_bot_safety_monitor/scripts/reach_goal_demo_success.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# Demo 脚本：启动 safety_aware 导航到单点，配合 safety_monitor 展示 pause/estop/recovery。请基于
# reach_goal.py 改写成带 SafetyState 监听的框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
#! /usr/bin/env python3

# ---------------------------------------------------------------------------
# 【场景】单点 reach_goal 演示
# 【演示脚本说明】Gazebo 联调用；print 便于录屏。需先起 safety_monitor + Nav2。
# ---------------------------------------------------------------------------


import math

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
import rclpy
from rclpy.duration import Duration

from sam_bot_safety_monitor.safety_navigation import SafetyAwareNavigator


DEMO_GOAL = (1.6, 0.0, 0.0)
FALLBACK_GOAL = (1.5, 0.8, 0.0)
PROGRESS_EPSILON = 0.08
STUCK_TIMEOUT_SEC = 12.0


def _create_goal_pose(navigator: SafetyAwareNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose



# 单点导航成功路径，不带 safety 干预
def main():
    rclpy.init()
    navigator = SafetyAwareNavigator(node_name="reach_goal_demo_success_navigator")

    print("[DEMO A] avoiding obstacle and navigating to reachable goal", flush=True)
    print(
        "[DEMO A] world spawn is fixed on the left side of the pillar and the goal is fixed on the right side",
        flush=True,
    )

    # 等 Nav2 生命周期到 active
    navigator.waitUntilNav2Active(localizer="smoother_server")
    print("[DEMO A] Nav2 active!", flush=True)

    goal_pose = _create_goal_pose(navigator, *DEMO_GOAL)
    print(
        "[DEMO A] reachable goal: "
        + f"({DEMO_GOAL[0]:.2f}, {DEMO_GOAL[1]:.2f}, {DEMO_GOAL[2]:.2f})",
        flush=True,
    )

    # 发一个固定 goal 验证链路
    if not navigator.goToPose(goal_pose):
        print("[DEMO A] goal request was rejected", flush=True)
        raise SystemExit(1)

    nav_start = navigator.get_clock().now()
    last_safety_state = None
    feedback_counter = 0
    best_distance_remaining = None
    last_progress_time = navigator.get_clock().now()
    fallback_triggered = False

    # spin 直到 goal 结束
    while rclpy.ok():
        task_complete = navigator.isTaskComplete()
        safety_state = navigator.get_safety_state_label()
        if safety_state != last_safety_state:
            print(f"[DEMO A] current safety state: {safety_state}", flush=True)
            last_safety_state = safety_state

        if navigator.is_waiting_for_recovery():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            continue

        if task_complete:
            break

        feedback_counter += 1
        feedback = navigator.getFeedback()
        if feedback:
            distance_remaining = getattr(feedback, "distance_remaining", None)
            if distance_remaining is not None:
                if (
                    best_distance_remaining is None
                    or distance_remaining < best_distance_remaining - PROGRESS_EPSILON
                ):
                    best_distance_remaining = distance_remaining
                    last_progress_time = navigator.get_clock().now()

            if feedback_counter % 5 == 0:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"[DEMO A] navigating around pillar, ETA {eta:.0f}s", flush=True)

            if (
                not fallback_triggered
                and navigator.get_clock().now() - last_progress_time
                > Duration(seconds=STUCK_TIMEOUT_SEC)
            ):
                print("[DEMO A] local planner stuck, fallback triggered", flush=True)
                navigator.cancelTask()
                fallback_pose = _create_goal_pose(navigator, *FALLBACK_GOAL)
                print(
                    "[DEMO A] switching to wider fallback goal: "
                    + f"({FALLBACK_GOAL[0]:.2f}, {FALLBACK_GOAL[1]:.2f}, {FALLBACK_GOAL[2]:.2f})",
                    flush=True,
                )
                if navigator.goToPose(fallback_pose):
                    fallback_triggered = True
                    nav_start = navigator.get_clock().now()
                    best_distance_remaining = None
                    last_progress_time = navigator.get_clock().now()

            if navigator.get_clock().now() - nav_start > Duration(seconds=180.0):
                print("[DEMO A] timeout reached, canceling goal", flush=True)
                navigator.cancelTask()

        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[DEMO A] goal succeeded", flush=True)
        print("[DEMO A] obstacle avoidance demo completed successfully", flush=True)
    elif result == TaskResult.CANCELED:
        print("[DEMO A] goal was canceled", flush=True)
        print("[DEMO A] obstacle avoidance demo did not complete", flush=True)
    elif result == TaskResult.FAILED:
        print("[DEMO A] goal failed", flush=True)
        print("[DEMO A] obstacle avoidance demo failed unexpectedly", flush=True)
    else:
        print("[DEMO A] invalid result state", flush=True)

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
