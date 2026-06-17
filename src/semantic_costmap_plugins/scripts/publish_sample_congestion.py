# ========================================================================
# 文件: src/semantic_costmap_plugins/scripts/publish_sample_congestion.py
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
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



class CongestionPublisher(Node):
    def __init__(self, topic: str, publish_period: float, once: bool) -> None:
        super().__init__("sample_congestion_publisher")
        self._publisher = self.create_publisher(Float32MultiArray, topic, 10)
        self._once = once
        self._published_once = False
        self._timer = self.create_timer(publish_period, self._on_timer)
        self.get_logger().info(
            f"Publishing sample congestion events on {topic} "
            f"(once={self._once}, period={publish_period:.2f}s)"
        )

    def _build_message(self) -> Float32MultiArray:
        msg = Float32MultiArray()
        # Format:
        #   x, y, radius, peak_cost, ttl_sec, exponent
        # You can concatenate multiple events in a single message.
        msg.data = [
            2.6, 0.0, 0.8, 140.0, 8.0, 1.6,
            4.4, -0.6, 0.7, 110.0, 6.0, 1.2,
        ]
        return msg

    def _on_timer(self) -> None:
        if self._once and self._published_once:
            return

        msg = self._build_message()
        self._publisher.publish(msg)
        self._published_once = True
        self.get_logger().info(f"Published {len(msg.data) // 6} congestion event(s)")

        if self._once:
            self.get_logger().info("Single-shot mode complete, shutting down in 1 second.")
            time.sleep(1.0)
            rclpy.shutdown()


# 演示用：往 congestion topic 打几帧事件
def main() -> None:
    parser = argparse.ArgumentParser(description="Publish example dynamic congestion events.")
    parser.add_argument("--topic", default="/semantic_congestion_events")
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = CongestionPublisher(args.topic, args.period, args.once)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
