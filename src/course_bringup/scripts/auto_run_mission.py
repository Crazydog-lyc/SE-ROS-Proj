#!/usr/bin/env python3

import sys

from course_interfaces.action import RunMission
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class AutoRunMission(Node):
    def __init__(self) -> None:
        super().__init__("auto_run_mission")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("max_retry_per_waypoint", 1)
        self.declare_parameter("allow_skip_waypoint", False)
        self.declare_parameter("server_timeout_sec", 120.0)
        self._client = ActionClient(self, RunMission, "/mission/run")

    def run(self) -> int:
        mission_file = str(self.get_parameter("mission_file").value)
        if not mission_file:
            self.get_logger().error("mission_file parameter is required")
            return 1

        timeout_sec = float(self.get_parameter("server_timeout_sec").value)
        self.get_logger().info(f"Waiting for /mission/run action server for up to {timeout_sec:.1f}s")
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Timed out waiting for /mission/run")
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
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
