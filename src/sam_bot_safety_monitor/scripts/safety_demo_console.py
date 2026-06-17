# ========================================================================
# 文件: src/sam_bot_safety_monitor/scripts/safety_demo_console.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# 在 sam_bot_safety_monitor 里帮我搭 SafetyMonitor 节点骨架（Python rclpy）：订阅 /scan、/odom，发布
# /safety/state（course_interfaces），提供 TriggerPause / TriggerEmergencyStop / TriggerCancel
# / RequestRecovery 服务，先把参数 declare、QoS、服务/话题注册和 monitor 定时器框架写好，检测逻辑我后面实现。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
#! /usr/bin/env python3

# ---------------------------------------------------------------------------
# 【演示脚本说明】Gazebo 联调用；print 便于录屏。需先起 safety_monitor + Nav2。
# ---------------------------------------------------------------------------


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


# 入口：初始化 navigator、发 goal、主循环 spin
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

# ---------------------------------------------------------------------------
# 【联调检查清单】
# 录屏时建议同时开 rqt 看 /safety/state 与 /mission/state
# 堵塞 demo 记得开 enable_blockage_monitor
# 1. safety_monitor 已 active 且 /scan /odom 有数据
# 2. Nav2 lifecycle 到 active，localizer 与 launch 一致
# 3. RViz 里能看到 robot 与 costmap
# 4. 故意触发 pause 时 mission_manager 应进入 PAUSED_FOR_SAFETY
# ---------------------------------------------------------------------------
