# ========================================================================
# 文件: src/sam_bot_safety_monitor/sam_bot_safety_monitor/__init__.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# 我在 ROS2 Humble 上做 Nav2 课程项目，需要新建 nav2_mission_manager 包。请帮我搭 Python Action Server
# 骨架：/mission/run 用 course_interfaces/RunMission，ReentrantCallbackGroup +
# MultiThreadedExecutor，发布 /mission/state，订阅 /safety/state；先把
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from .safety_monitor import SafetyMonitor
from .safety_navigation import SafetyAwareNavigator

__all__ = ["SafetyAwareNavigator", "SafetyMonitor"]

