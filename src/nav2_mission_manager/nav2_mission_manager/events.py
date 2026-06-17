# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/events.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 我在 ROS2 Humble 上做 Nav2 课程项目，需要新建 nav2_mission_manager 包。请帮我搭 Python Action Server
# 骨架：/mission/run 用 course_interfaces/RunMission，ReentrantCallbackGroup +
# MultiThreadedExecutor，发布 /mission/state，订阅 /safety/state；先把
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from dataclasses import dataclass
from enum import Enum



class EventType(str, Enum):
    MISSION_REQUESTED = "MISSION_REQUESTED"
    MISSION_LOADED = "MISSION_LOADED"
    MISSION_INVALID = "MISSION_INVALID"
    NAV2_READY = "NAV2_READY"
    NAV2_NOT_READY = "NAV2_NOT_READY"
    GOAL_SENT = "GOAL_SENT"
    GOAL_REJECTED = "GOAL_REJECTED"
    FEEDBACK_TICK = "FEEDBACK_TICK"
    GOAL_SUCCEEDED = "GOAL_SUCCEEDED"
    GOAL_FAILED = "GOAL_FAILED"
    GOAL_TIMEOUT = "GOAL_TIMEOUT"
    SAFETY_STOP = "SAFETY_STOP"
    SAFETY_CLEAR = "SAFETY_CLEAR"
    ACTION_CANCEL_REQUESTED = "ACTION_CANCEL_REQUESTED"
    ACTION_CANCEL_CONFIRMED = "ACTION_CANCEL_CONFIRMED"


@dataclass(frozen=True)
class Event:
    type: EventType
    reason: str = ""
