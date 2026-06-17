# ========================================================================
# 文件: src/semantic_costmap_plugins/scripts/set_task_mode.py
# 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
# ========================================================================
#
# 【AI-PROMPT】
# 基于 Nav2 Humble costmap_2d::Layer，帮我新建 semantic_costmap_plugins 包骨架：SemanticZoneLayer /
# PreferredLaneLayer / DynamicCongestionLayer 三个插件类，继承 CostmapLayer，先实现
# onInitialize、updateBounds、updateCosts 空壳和 pluginlib 导出，附带 geometry_utils、cost_functions
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class TaskModePublisher(Node):
    def __init__(self, topic: str, mode: str) -> None:
        super().__init__("task_mode_publisher")
        self._publisher = self.create_publisher(String, topic, 10)
        self._mode = mode
        self._timer = self.create_timer(0.5, self._on_timer)
        self._count = 0
        self.get_logger().info(f"Will publish task mode '{mode}' on {topic}")

    def _on_timer(self) -> None:
        msg = String()
        msg.data = self._mode
        self._publisher.publish(msg)
        self._count += 1
        self.get_logger().info(f"Published task mode '{self._mode}' ({self._count}/5)")
        if self._count >= 5:
            self.get_logger().info("Done publishing task mode.")
            rclpy.shutdown()


# 发布 /semantic_task_mode 切换 costmap 模式
def main() -> None:
    parser = argparse.ArgumentParser(description="Publish a semantic task mode string.")
    parser.add_argument("mode", help="Task mode to publish, for example: all, patrol, delivery, charge")
    parser.add_argument("--topic", default="/semantic_task_mode")
    args = parser.parse_args()

    rclpy.init()
    node = TaskModePublisher(args.topic, args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
