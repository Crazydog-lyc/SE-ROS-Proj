# ========================================================================
# 文件: src/course_bringup/scripts/auto_run_mission.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 写一个 auto_run_mission CLI 节点：ActionClient 连 /mission/run，从参数读 mission_file，send_goal 后
# spin 等 result，打印 feedback。请生成 Node 类和 main 入口框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
#!/usr/bin/env python3
# 【集成脚本】course 全栈联调辅助

import sys

from course_interfaces.action import RunMission
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node



# CLI 封装 /mission/run，batch 跑 case 时用
class AutoRunMission(Node):
    def __init__(self) -> None:
        super().__init__("auto_run_mission")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("max_retry_per_waypoint", 1)
        self.declare_parameter("allow_skip_waypoint", False)
        self.declare_parameter("server_timeout_sec", 120.0)
        self._client = ActionClient(self, RunMission, "/mission/run")

    # 等 server -> send_goal -> 等 result
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
        # 路径来自 launch 参数或命令行
goal.mission_file = mission_file
        goal.max_retry_per_waypoint = int(
            self.get_parameter("max_retry_per_waypoint").value
        )
        goal.allow_skip_waypoint = bool(
            self.get_parameter("allow_skip_waypoint").value
        )

        self.get_logger().info(f"Sending mission goal for {mission_file}")
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        # 阻塞等到 goal 被 accept
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Mission goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        # 阻塞等到 mission 终态
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

    # 把 Action feedback 打到日志，便于对照 RViz
def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Mission feedback: "
            f"waypoint={feedback.current_waypoint_index} "
            f"state={feedback.current_state} "
            f"elapsed={feedback.elapsed_sec:.1f}s"
        )


# 入口：初始化 navigator、发 goal、主循环 spin
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
