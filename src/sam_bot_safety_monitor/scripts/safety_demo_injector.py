#! /usr/bin/env python3

import copy
from math import hypot
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class SafetyDemoInjector(Node):
    """Injects a temporary odom freeze for a stable blockage demo."""

    def __init__(self) -> None:
        super().__init__("safety_demo_injector")
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("output_odom_topic", "/safety_demo/odom")
        self.declare_parameter("fault_delay_sec", 8.0)
        self.declare_parameter("fault_duration_sec", 4.0)
        self.declare_parameter("arm_distance_m", 0.2)

        self.input_odom_topic = self.get_parameter("input_odom_topic").value
        self.output_odom_topic = self.get_parameter("output_odom_topic").value
        self.fault_delay_sec = float(self.get_parameter("fault_delay_sec").value)
        self.fault_duration_sec = float(self.get_parameter("fault_duration_sec").value)
        self.arm_distance_m = float(self.get_parameter("arm_distance_m").value)

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        status_qos.reliability = ReliabilityPolicy.RELIABLE

        self.status_pub = self.create_publisher(
            String, "/safety_demo/status", status_qos
        )
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.create_subscription(
            Odometry, self.input_odom_topic, self._odom_callback, 10
        )

        self.last_odom = None
        self.start_pose = None
        self.motion_started_at = None
        self.freeze_announced = False
        self.resume_announced = False

        self._publish_status("safety demo injector started")

    def _odom_callback(self, msg: Odometry) -> None:
        if self.last_odom is None:
            self.last_odom = copy.deepcopy(msg)

        position = msg.pose.pose.position
        current_xy = (position.x, position.y)
        if self.start_pose is None:
            self.start_pose = current_xy

        if self.motion_started_at is None:
            distance = hypot(
                current_xy[0] - self.start_pose[0], current_xy[1] - self.start_pose[1]
            )
            if distance >= self.arm_distance_m:
                self.motion_started_at = time.monotonic()
                self._publish_status(
                    "navigation motion detected, arming blockage injection"
                )

        freeze_active = False
        if self.motion_started_at is not None:
            elapsed = time.monotonic() - self.motion_started_at
            freeze_active = self.fault_delay_sec <= elapsed < (
                self.fault_delay_sec + self.fault_duration_sec
            )

        if freeze_active:
            if not self.freeze_announced:
                self.freeze_announced = True
                self._publish_status("freezing odom relay to trigger blockage detection")

            frozen = copy.deepcopy(self.last_odom)
            frozen.header.stamp = msg.header.stamp
            self.odom_pub.publish(frozen)
            return

        if self.freeze_announced and not self.resume_announced:
            self.resume_announced = True
            self._publish_status("restoring odom relay after blockage injection")

        self.last_odom = copy.deepcopy(msg)
        self.odom_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))


def main() -> None:
    rclpy.init()
    node = SafetyDemoInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
