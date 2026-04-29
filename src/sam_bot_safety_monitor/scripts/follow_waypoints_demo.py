#! /usr/bin/env python3

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


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a safety demo with explicit post-recovery full mission replay."
    )
    parser.add_argument("--restart-anchor-x", type=float, default=-1.6)
    parser.add_argument("--restart-anchor-y", type=float, default=0.0)
    parser.add_argument("--restart-anchor-yaw", type=float, default=1.05)
    parser.add_argument("--navigation-timeout", type=float, default=900.0)
    return parser.parse_args(remove_ros_args(args=None)[1:])


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


def _build_demo_route(navigator: SafetyAwareNavigator):
    return [_create_pose(navigator, x, y, yaw) for x, y, yaw in DEMO_ROUTE]


def _build_restart_anchor_pose(
    navigator: SafetyAwareNavigator, x: float, y: float, yaw: float
) -> PoseStamped:
    return _create_pose(navigator, x, y, yaw)


def _cancel_active_task_if_needed(navigator: SafetyAwareNavigator) -> None:
    if navigator.result_future is not None:
        navigator.cancelTask()


def _start_anchor_return(
    navigator: SafetyAwareNavigator, restart_anchor_pose: PoseStamped
) -> bool:
    print("returning to restart anchor pose", flush=True)
    return navigator.goToPose(restart_anchor_pose)


def _start_full_mission_replay(
    navigator: SafetyAwareNavigator, original_goals
) -> bool:
    return navigator.followWaypoints(original_goals)


def main():
    args = parse_args()
    rclpy.init()
    navigator = SafetyAwareNavigator(
        node_name="follow_waypoints_demo_navigator",
        recovery_mode="client_replay_after_normal",
    )

    print("navigation started", flush=True)
    print("demo route: 8-waypoint ring loop around the room", flush=True)
    print(f"recovery mode: {navigator.get_recovery_mode()}", flush=True)

    navigator.waitUntilNav2Active(localizer="smoother_server")
    print("Nav2 active!", flush=True)

    original_goals = _build_demo_route(navigator)
    restart_anchor_pose = _build_restart_anchor_pose(
        navigator,
        args.restart_anchor_x,
        args.restart_anchor_y,
        args.restart_anchor_yaw,
    )

    print(f"demo route ready with {len(original_goals)} waypoints", flush=True)
    print(
        "restart anchor pose: "
        + f"({args.restart_anchor_x:.2f}, {args.restart_anchor_y:.2f}, {args.restart_anchor_yaw:.2f})",
        flush=True,
    )

    nav_start = navigator.get_clock().now()
    if not navigator.followWaypoints(original_goals):
        print(
            "Waypoint mission was rejected because safety monitor is not ready for navigation.",
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

    while rclpy.ok():
        task_complete = navigator.isTaskComplete()
        safety_state = navigator.get_safety_state_label()

        if safety_state != last_safety_state:
            print(f"Current safety state: {safety_state}", flush=True)
            if safety_state in ("PAUSED", "EMERGENCY_STOP", "RECOVERING"):
                print(
                    "auto-detected safety event: "
                    + f"{navigator.get_safety_source()} - {navigator.get_safety_reason()}",
                    flush=True,
                )
            if safety_state == "PAUSED":
                print("safety state changed to PAUSED", flush=True)
                print("waiting for recovery...", flush=True)
            elif safety_state == "EMERGENCY_STOP":
                print("safety state changed to EMERGENCY_STOP", flush=True)
            elif safety_state == "RECOVERING":
                was_recovering = True
                print("safety state changed to RECOVERING", flush=True)
                print("waiting for demo replay after recovery", flush=True)
            elif safety_state == "NORMAL":
                if was_recovering and replay_after_recovery_enabled and not replay_done:
                    print("detected recovery completion", flush=True)
                if last_safety_state == "RECOVERING":
                    print("navigation resumed", flush=True)
            elif safety_state == "CANCELED":
                print("exiting because task is canceled", flush=True)
            last_safety_state = safety_state

        if navigator.is_waiting_for_recovery():
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

            print("anchor return failed, replaying from current pose", flush=True)
            if _start_full_mission_replay(navigator, original_goals):
                anchor_return_active = False
                full_mission_replay_active = True
                nav_start = navigator.get_clock().now()
                print("full mission replay started", flush=True)
                rclpy.spin_once(navigator, timeout_sec=0.1)
                continue

            print("full mission replay request was rejected", flush=True)
            demo_failed = True
            break

        if task_complete:
            if anchor_return_active:
                anchor_return_active = False
                if navigator.status == GoalStatus.STATUS_SUCCEEDED:
                    print("anchor reached, replaying full mission", flush=True)
                else:
                    print("anchor return failed, replaying from current pose", flush=True)

                if _start_full_mission_replay(navigator, original_goals):
                    full_mission_replay_active = True
                    nav_start = navigator.get_clock().now()
                    print("full mission replay started", flush=True)
                    rclpy.spin_once(navigator, timeout_sec=0.1)
                    continue

                print("full mission replay request was rejected", flush=True)
                demo_failed = True
                break

            if full_mission_replay_active:
                if navigator.status == GoalStatus.STATUS_SUCCEEDED:
                    print("full mission replay completed", flush=True)
                full_mission_replay_active = False
                replay_in_progress = False
                break

            if replay_in_progress:
                replay_in_progress = False

            break

        feedback_counter += 1
        feedback = navigator.getFeedback()
        if feedback and feedback_counter % 5 == 0 and not anchor_return_active:
            print(
                "Executing current waypoint: "
                + f"{navigator.get_current_waypoint_index() + 1}/{len(original_goals)}",
                flush=True,
            )
            if navigator.get_clock().now() - nav_start > Duration(
                seconds=args.navigation_timeout
            ):
                print("navigation canceled / failed", flush=True)
                navigator.cancelTask()

        rclpy.spin_once(navigator, timeout_sec=0.1)

    if demo_failed:
        print("navigation canceled / failed", flush=True)
        raise SystemExit(1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!", flush=True)
        if replay_done:
            print("navigation finished after full mission replay", flush=True)
        else:
            print("navigation finished at final demo waypoint", flush=True)
    elif result == TaskResult.CANCELED:
        if navigator.is_terminal_canceled():
            print("Goal was canceled!", flush=True)
            print("navigation canceled / failed", flush=True)
        else:
            print("Goal was canceled without a terminal safety cancel state!", flush=True)
    elif result == TaskResult.FAILED:
        print("Goal failed!", flush=True)
        print("navigation canceled / failed", flush=True)
    else:
        print("Goal has an invalid return status!", flush=True)

    raise SystemExit(0)


if __name__ == "__main__":
    main()
