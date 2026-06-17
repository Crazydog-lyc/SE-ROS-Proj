# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/models.py
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
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .states import MissionState



class NavTaskResult(str, Enum):
    # 和 nav2_simple_commander TaskResult 对齐，方便 adapter 映射
    SUCCEEDED = "SUCCEEDED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


class TransitionCommand(str, Enum):
    # 状态机 handler 返回后，Action Server 据此调 navigator
    NONE = "NONE"
    WAIT_FOR_NAV2 = "WAIT_FOR_NAV2"
    SEND_GOAL = "SEND_GOAL"
    CANCEL_GOAL = "CANCEL_GOAL"
    COMPLETE_MISSION = "COMPLETE_MISSION"
    FAIL_MISSION = "FAIL_MISSION"


@dataclass(frozen=True)
class WaypointSpec:
    waypoint_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionSpec:
    mission_id: str
    frame_id: str
    waypoints: list[WaypointSpec]


@dataclass
class MissionFeedbackSnapshot:
    current_waypoint_index: int = 0
    distance_remaining: Optional[float] = None
    navigation_time_sec: Optional[float] = None
    estimated_time_remaining_sec: Optional[float] = None


@dataclass
class MissionExecutionContext:
    # 一次 /mission/run goal 的全局上下文，状态机只改这个对象
    mission_spec: Optional[MissionSpec] = None
    state: MissionState = MissionState.IDLE
    max_retry_per_waypoint: int = 1
    allow_skip_waypoint: bool = False
    current_index: int = 0
    completed_count: int = 0
    failed_count: int = 0
    retry_count: int = 0
    safety_stop_count: int = 0
    last_reason: str = ""
    current_goal_start_monotonic: Optional[float] = None
    current_feedback: Optional[MissionFeedbackSnapshot] = None
    metadata: dict[str, str] = field(default_factory=dict)

    @property
    def total_waypoints(self) -> int:
        return len(self.mission_spec.waypoints) if self.mission_spec else 0

    @property
    def current_waypoint(self) -> WaypointSpec:
        if self.mission_spec is None:
            raise RuntimeError("Mission has not been loaded.")
        return self.mission_spec.waypoints[self.current_index]

    @property
    def mission_id(self) -> str:
        return self.mission_spec.mission_id if self.mission_spec else ""

    @property
    def progress_percent(self) -> float:
        # 按已完成 waypoint 数算百分比，给 /mission/state 用
        if self.total_waypoints == 0:
            return 0.0
        return (self.completed_count / self.total_waypoints) * 100.0


@dataclass(frozen=True)
class StateTransition:
    new_state: MissionState
    command: TransitionCommand = TransitionCommand.NONE
    reason: str = ""
