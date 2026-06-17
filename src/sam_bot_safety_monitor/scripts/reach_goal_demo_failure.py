# ========================================================================
# 文件: src/sam_bot_safety_monitor/scripts/reach_goal_demo_failure.py
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
from course_interfaces.srv import RequestRecovery

from sam_bot_safety_monitor.safety_navigation import SafetyAwareNavigator


DEMO_GOAL = (0.75, 0.75, 0.0)


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

# 故意发到 unreachable 点，看 Nav2 失败反馈
def main():
    rclpy.init()
    navigator = SafetyAwareNavigator(node_name="reach_goal_demo_failure_navigator")

    print("[DEMO B] safety stop demonstration near obstacle", flush=True)
    print(
        "[DEMO B] goal is beyond the pillar on the upper-right; safety monitor should pause before collision",
        flush=True,
    )

    navigator.waitUntilNav2Active(localizer="smoother_server")
    print("[DEMO B] Nav2 active!", flush=True)

    recovery_client = navigator.create_client(
        RequestRecovery, "/safety/request_recovery"
    )
    if not recovery_client.wait_for_service(timeout_sec=10.0):
        print("[DEMO B] safety recovery service is unavailable", flush=True)
        raise SystemExit(1)

    if navigator.get_safety_state_label() != "NORMAL":
        print(
            f"[DEMO B] safety state is {navigator.get_safety_state_label()} at startup, requesting recovery reset",
            flush=True,
        )
        recovery_request = RequestRecovery.Request()
        recovery_request.strategy = "replan"
        recovery_request.reason = "DEMO B startup reset to NORMAL"
        future = recovery_client.call_async(recovery_request)
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
        reset_deadline = navigator.get_clock().now() + Duration(seconds=5.0)
        while (
            rclpy.ok()
            and navigator.get_safety_state_label() != "NORMAL"
            and navigator.get_clock().now() < reset_deadline
        ):
            rclpy.spin_once(navigator, timeout_sec=0.1)

    if navigator.get_safety_state_label() != "NORMAL":
        print(
            f"[DEMO B] safety state remained {navigator.get_safety_state_label()}, cannot start demo",
            flush=True,
        )
        raise SystemExit(1)

    goal_pose = _create_goal_pose(navigator, *DEMO_GOAL)
    print(
        "[DEMO B] obstacle-side goal: "
        + f"({DEMO_GOAL[0]:.2f}, {DEMO_GOAL[1]:.2f}, {DEMO_GOAL[2]:.2f})",
        flush=True,
    )

    # 目标点在墙里或地图外
    if not navigator.goToPose(goal_pose):
        print("[DEMO B] goal request was rejected immediately", flush=True)
        raise SystemExit(1)

    nav_start = navigator.get_clock().now()
    last_safety_state = None
    feedback_counter = 0
    demo_pause_completed = False

    # 等 FAILED 结果
    while rclpy.ok():
        task_complete = navigator.isTaskComplete()
        safety_state = navigator.get_safety_state_label()
        if safety_state != last_safety_state:
            print(f"[DEMO B] current safety state: {safety_state}", flush=True)
            last_safety_state = safety_state

        if safety_state == "PAUSED":
            print(
                "[DEMO B] safety monitor paused the robot near the unreachable obstacle target",
                flush=True,
            )
            demo_pause_completed = True
            break

        if navigator.is_waiting_for_recovery():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            continue

        if task_complete:
            break

        feedback_counter += 1
        feedback = navigator.getFeedback()
        if feedback and feedback_counter % 5 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f"[DEMO B] approaching pillar-side goal, ETA {eta:.0f}s", flush=True)

        if navigator.get_clock().now() - nav_start > Duration(seconds=90.0):
            print(
                "[DEMO B] safety-stop demo timed out before monitor intervention, canceling task",
                flush=True,
            )
            navigator.cancelTask()

        rclpy.spin_once(navigator, timeout_sec=0.1)

    if demo_pause_completed:
        print("[DEMO B] safety-stop demo completed with obstacle pause handling", flush=True)
        raise SystemExit(0)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[DEMO B] goal unexpectedly succeeded", flush=True)
    elif result == TaskResult.CANCELED:
        print("[DEMO B] goal was canceled", flush=True)
        print("[DEMO B] safety-stop demo ended via cancel handling", flush=True)
    elif result == TaskResult.FAILED:
        print("[DEMO B] goal failed", flush=True)
        print("[DEMO B] safety-stop demo completed via planner/controller failure", flush=True)
    else:
        print("[DEMO B] invalid result state", flush=True)

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
