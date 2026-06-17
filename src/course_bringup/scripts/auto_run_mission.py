#!/usr/bin/python3.10
# ========================================================================
# 文件: src/course_bringup/scripts/auto_run_mission.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
# 【集成脚本】scenario batch 用：等 Nav2 active 后再发 /mission/run

import time

from course_interfaces.action import RunMission
import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException


class AutoRunMission(Node):
    def __init__(self) -> None:
        super().__init__("auto_run_mission")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("max_retry_per_waypoint", 1)
        self.declare_parameter("allow_skip_waypoint", False)
        self.declare_parameter("server_timeout_sec", 420.0)
        self.declare_parameter("nav2_ready_timeout_sec", 420.0)
        self.declare_parameter("nav2_localizer", "smoother_server")
        self._client = ActionClient(self, RunMission, "/mission/run")
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _wait_for_transform(self, target_frame: str, source_frame: str, deadline: float) -> bool:
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            try:
                if self._tf_buffer.can_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.5),
                ):
                    return True
            except TransformException:
                pass
            try:
                rclpy.spin_once(self, timeout_sec=0.2)
            except (KeyboardInterrupt, RuntimeError):
                return False
        return False

    def _wait_for_navigation_frames(self, deadline: float) -> bool:
        for target, source, label in (
            ("base_link", "odom", "odom->base_link"),
            ("base_link", "map", "map->base_link"),
        ):
            if not self._wait_for_transform(target, source, deadline):
                self.get_logger().error(f"Timed out waiting for TF {label}")
                return False
            self.get_logger().info(f"TF ready: {label}")
        return True

    def _wait_lifecycle_active(self, node_name: str, deadline: float) -> bool:
        client = self.create_client(GetState, f"{node_name}/get_state")
        req = GetState.Request()
        while time.monotonic() < deadline:
            if not rclpy.ok():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            if not client.wait_for_service(timeout_sec=min(2.0, remaining)):
                try:
                    rclpy.spin_once(self, timeout_sec=0.2)
                except (KeyboardInterrupt, RuntimeError):
                    return False
                continue
            future = client.call_async(req)
            try:
                rclpy.spin_until_future_complete(self, future, timeout_sec=min(2.0, remaining))
            except (KeyboardInterrupt, RuntimeError):
                return False
            if future.result() is not None and future.result().current_state.label == "active":
                return True
            try:
                rclpy.spin_once(self, timeout_sec=0.5)
            except (KeyboardInterrupt, RuntimeError):
                return False
        return False

    def _wait_for_nav2(self) -> bool:
        timeout_sec = float(self.get_parameter("nav2_ready_timeout_sec").value)
        localizer = str(self.get_parameter("nav2_localizer").value)
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info(
            f"Waiting for Nav2 ({localizer}, bt_navigator) to become active "
            f"for up to {timeout_sec:.0f}s ..."
        )
        for node_name in (localizer, "bt_navigator"):
            if not self._wait_lifecycle_active(node_name, deadline):
                self.get_logger().error(f"Timed out waiting for {node_name} to become active")
                return False
            self.get_logger().info(f"{node_name} is active")
        return True

    def run(self) -> int:
        mission_file = str(self.get_parameter("mission_file").value)
        if not mission_file:
            self.get_logger().error("mission_file parameter is required")
            return 1

        timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.get_logger().info(
            f"Waiting for /mission/run action server for up to {timeout_sec:.1f}s"
        )
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Timed out waiting for /mission/run")
            return 1

        deadline = time.monotonic() + float(self.get_parameter("nav2_ready_timeout_sec").value)
        if not self._wait_for_navigation_frames(deadline):
            return 1

        if not self._wait_for_nav2():
            return 1

        goal = RunMission.Goal()
        goal.mission_file = mission_file
        goal.max_retry_per_waypoint = int(
            self.get_parameter("max_retry_per_waypoint").value
        )
        goal.allow_skip_waypoint = bool(
            self.get_parameter("allow_skip_waypoint").value
        )

        self.get_logger().info(f"Sending mission goal for {mission_file}")
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Mission goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            self.get_logger().error("Mission action returned no result")
            return 1

        result = result_wrapper.result
        if result.success:
            self.get_logger().info(
                f"Mission succeeded: completed_waypoints={result.completed_waypoints}"
            )
            return 0

        self.get_logger().error(
            f"Mission failed: final_state={result.final_state} message={result.message}"
        )
        return 1

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Mission feedback: "
            f"waypoint={feedback.current_waypoint_index} "
            f"state={feedback.current_state} "
            f"elapsed={feedback.elapsed_sec:.1f}s"
        )


def main() -> int:
    rclpy.init()
    node = AutoRunMission()
    try:
        return node.run()
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
