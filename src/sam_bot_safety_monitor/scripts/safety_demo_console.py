#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from course_interfaces.msg import SafetyState


class SafetyDemoConsole(Node):
    """Headless-friendly console output for safety demo state."""

    def __init__(self) -> None:
        super().__init__("safety_demo_console")
        self.declare_parameter("heartbeat_period_sec", 2.0)

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.last_state_line = None
        self.last_demo_status = None
        self.latest_state = None

        self.create_subscription(
            SafetyState, "/safety/state", self._state_callback, state_qos
        )
        self.create_subscription(String, "/safety_demo/status", self._status_callback, 10)
        heartbeat_period = float(self.get_parameter("heartbeat_period_sec").value)
        self.create_timer(heartbeat_period, self._heartbeat_callback)

        self.get_logger().info("Safety demo console started")

    def _state_callback(self, msg: SafetyState) -> None:
        self.latest_state = msg
        state_line = self._format_state_line(prefix="[SAFETY]")
        if state_line != self.last_state_line:
            print(state_line, flush=True)
            self.last_state_line = state_line

    def _status_callback(self, msg: String) -> None:
        if msg.data != self.last_demo_status:
            print(f"[DEMO] {msg.data}", flush=True)
            self.last_demo_status = msg.data

    def _heartbeat_callback(self) -> None:
        if self.latest_state is None:
            print("[SAFETY_HEARTBEAT] waiting for /safety/state ...", flush=True)
            return
        print(self._format_state_line(prefix="[SAFETY_HEARTBEAT]"), flush=True)

    def _format_state_line(self, prefix: str) -> str:
        if self.latest_state is None:
            return f"{prefix} no safety state received yet"

        msg = self.latest_state
        return (
            f"{prefix} state={msg.state_label} source={msg.source} "
            f"reason={msg.reason} detail={msg.detail} "
            f"recovery_attempts={msg.recovery_attempts}"
        )


def main() -> None:
    rclpy.init()
    node = SafetyDemoConsole()
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
