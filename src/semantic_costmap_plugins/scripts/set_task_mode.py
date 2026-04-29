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
