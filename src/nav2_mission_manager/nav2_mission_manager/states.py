# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/states.py
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
from enum import Enum



class MissionState(str, Enum):
    IDLE = "IDLE"
    LOADING_MISSION = "LOADING_MISSION"
    WAITING_FOR_NAV2 = "WAITING_FOR_NAV2"
    DISPATCHING_GOAL = "DISPATCHING_GOAL"
    WAITING_FOR_RESULT = "WAITING_FOR_RESULT"
    CANCELING_FOR_SAFETY = "CANCELING_FOR_SAFETY"
    PAUSED_FOR_SAFETY = "PAUSED_FOR_SAFETY"
    RETRYING_GOAL = "RETRYING_GOAL"
    SKIPPING_GOAL = "SKIPPING_GOAL"
    MISSION_SUCCEEDED = "MISSION_SUCCEEDED"
    MISSION_FAILED = "MISSION_FAILED"
    MISSION_CANCELED = "MISSION_CANCELED"
