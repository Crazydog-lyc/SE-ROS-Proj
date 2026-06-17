# ========================================================================
# 文件: src/sam_bot_safety_monitor/scripts/safety_monitor_node.py
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

from sam_bot_safety_monitor.safety_monitor import SafetyMonitor



# 入口：初始化 navigator、发 goal、主循环 spin
def main() -> None:
    rclpy.init()
    node = SafetyMonitor()
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
