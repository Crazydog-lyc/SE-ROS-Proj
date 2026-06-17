# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/mission_action_server.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# MissionActionServerNode：ActionServer 执行循环里 poll safety state、goal timeout、调用
# state_machine.handle_event，TransitionCommand 分支调 navigator。请生成
# _process_event、_execute_callback、_publish_state 框架，业务分支我后面填。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
# 【实现说明】显式状态机 + Action Server 驱动 Nav2 waypoint 序列，订阅 /safety/state。
import threading
import time
from typing import Callable, Optional, Tuple

from course_interfaces.action import RunMission
from course_interfaces.msg import MissionState, SafetyState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .events import Event, EventType
from .mission_loader import MissionLoadError, load_mission_file
from .models import MissionExecutionContext, NavTaskResult, TransitionCommand
from .navigator_adapter import BasicNavigatorAdapter
from .state_machine import MissionStateMachine
from .states import MissionState as MissionStateEnum



# TODO[徐梓鸣]：FR-B-02 实现 /mission/run Action Server，驱动 waypoint 序列并响应安全状态
class MissionActionServerNode(Node):
    #         # Reentrant 回调组：execute 循环里能收 safety 订阅
    def __init__(self, navigator_factory: Optional[Callable[[], BasicNavigatorAdapter]] = None):
        # TODO[徐梓鸣]：FR-B-06 周期性发布 /mission/state 供 RViz/脚本监控
        super().__init__("mission_action_server")
        self._callback_group = ReentrantCallbackGroup()
        self._navigator_factory = navigator_factory or self._default_navigator_factory
        self._goal_lock = threading.Lock()
        self._safety_lock = threading.Lock()
        self._active_goal = False
        self._latest_safety_msg: Optional[SafetyState] = None
        self._latest_safe_since: Optional[float] = None

        self.declare_parameter("nav2_ready_timeout_sec", 60.0)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("feedback_poll_period_sec", 0.2)
        self.declare_parameter("mission_state_publish_period_sec", 0.5)
        self.declare_parameter("default_max_retry_per_waypoint", 1)
        self.declare_parameter("default_allow_skip_waypoint", False)
        self.declare_parameter("safety_clear_hold_sec", 1.0)
        self.declare_parameter("log_feedback_every_n_ticks", 5)
        self.declare_parameter("nav2_localizer", "smoother_server")

        self._mission_state_pub = self.create_publisher(MissionState, "/mission/state", 10)
        self.create_subscription(
            SafetyState,
            "/safety/state",
            self._on_safety_state,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            RunMission,
            "/mission/run",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

    def destroy_node(self) -> bool:
        # TODO[徐梓鸣]：FR-B-05 WAITING_FOR_RESULT 中订阅 /safety/state，STOP_NOW 触发 SAFETY_STOP
        self._action_server.destroy()
        return super().destroy_node()

    def _default_navigator_factory(self) -> BasicNavigatorAdapter:
        # TODO[徐梓鸣]：FR-B-05 PAUSED_FOR_SAFETY 中等待 safety_clear_hold_sec 后重新派发 waypoint
        localizer = self.get_parameter("nav2_localizer").get_parameter_value().string_value
        return BasicNavigatorAdapter(localizer=localizer)

    #         # 同时只跑一个 mission
    def _goal_callback(self, goal_request: RunMission.Goal) -> GoalResponse:
        # TODO[徐梓鸣]：FR-B-04 goal 超时与 Nav2 结果映射到状态机事件
        del goal_request
        with self._goal_lock:
            if self._active_goal:
                self.get_logger().warn("[MISSION][GOAL][M001] Rejecting new goal because one is already active.")
                return GoalResponse.REJECT
            self._active_goal = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _on_safety_state(self, msg: SafetyState) -> None:
        with self._safety_lock:
            self._latest_safety_msg = msg
            # 记录 SAFE 持续了多久，避免安全层抖动导致 mission 立刻重发 goal
            if msg.level == SafetyState.SAFE:
                if self._latest_safe_since is None:
                    self._latest_safe_since = time.monotonic()
            else:
                self._latest_safe_since = None

    def _get_safety_snapshot(self) -> Tuple[Optional[SafetyState], Optional[float]]:
        with self._safety_lock:
            return self._latest_safety_msg, self._latest_safe_since

    def _is_safety_clear_held(self, hold_sec: float) -> bool:
        with self._safety_lock:
            msg = self._latest_safety_msg
            safe_since = self._latest_safe_since
        if msg is None or msg.level != SafetyState.SAFE or safe_since is None:
            return False
        return time.monotonic() - safe_since >= hold_sec

    def _publish_state(self, context: MissionExecutionContext) -> None:
        # 给 RViz 面板和 auto_run_mission 脚本看进度
        msg = MissionState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mission_id = context.mission_id
        msg.current_waypoint_index = min(context.current_index, context.total_waypoints)
        msg.total_waypoints = context.total_waypoints
        msg.state = context.state.value
        msg.progress_percent = float(context.progress_percent)
        msg.detail = context.last_reason
        self._mission_state_pub.publish(msg)

    def _process_event(
        self,
        machine: MissionStateMachine,
        context: MissionExecutionContext,
        navigator: BasicNavigatorAdapter,
        event: Event,
    ) -> None:
        # 状态机可能一次事件触发多个 TransitionCommand，用 while 链式处理
        next_event = event
        while next_event is not None:
            transition = machine.handle_event(context, next_event)
            self._publish_state(context)
            self.get_logger().info(
                f"[MISSION][{context.state.value}][M010] {context.last_reason or 'State updated.'}"
            )

            if transition.command == TransitionCommand.WAIT_FOR_NAV2:
                timeout = self.get_parameter("nav2_ready_timeout_sec").value
                ready = navigator.wait_until_ready(float(timeout))
                next_event = Event(
                    EventType.NAV2_READY if ready else EventType.NAV2_NOT_READY,
                    "Nav2 is ready." if ready else "Nav2 did not become ready before timeout.",
                )
                continue

            if transition.command == TransitionCommand.SEND_GOAL:
                context.current_goal_start_monotonic = time.monotonic()
                sent = navigator.send_goal(
                    context.current_waypoint,
                    frame_id=context.mission_spec.frame_id if context.mission_spec else "map",
                )
                next_event = Event(
                    EventType.GOAL_SENT if sent else EventType.GOAL_REJECTED,
                    f"Dispatching waypoint {context.current_index + 1}/{context.total_waypoints}."
                    if sent
                    else "Navigator rejected the goal request.",
                )
                continue

            if transition.command == TransitionCommand.CANCEL_GOAL:
                navigator.cancel_task()
                # 最多等 5s，确认 Nav2 真的 cancel 完再进 PAUSED
                deadline = time.monotonic() + 5.0
                poll_period = float(self.get_parameter("feedback_poll_period_sec").value)
                while time.monotonic() < deadline and navigator.task_active and not navigator.is_task_complete():
                    time.sleep(poll_period)
                next_event = Event(
                    EventType.ACTION_CANCEL_CONFIRMED,
                    "Navigation task canceled.",
                )
                continue

            next_event = None

    #         # Action 主线程：load mission + 轮询 Nav2
    def _execute_callback(self, goal_handle) -> RunMission.Result:
        try:
            navigator = self._navigator_factory()
            machine = MissionStateMachine()
            request = goal_handle.request
            context = MissionExecutionContext(
                max_retry_per_waypoint=int(request.max_retry_per_waypoint),
                allow_skip_waypoint=bool(request.allow_skip_waypoint),
            )

            self._process_event(
                machine,
                context,
                navigator,
                Event(EventType.MISSION_REQUESTED, "Mission request accepted."),
            )

            try:
                context.mission_spec = load_mission_file(request.mission_file)
            except MissionLoadError as exc:
                self._process_event(machine, context, navigator, Event(EventType.MISSION_INVALID, str(exc)))
                goal_handle.abort()
                return self._build_result(context, success=False)

            self._process_event(
                machine,
                context,
                navigator,
                Event(EventType.MISSION_LOADED, "Mission file loaded successfully."),
            )

            feedback_counter = 0
            feedback_poll_period = float(self.get_parameter("feedback_poll_period_sec").value)
            goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
            safety_hold_sec = float(self.get_parameter("safety_clear_hold_sec").value)
            last_state_publish = 0.0
            state_publish_period = float(self.get_parameter("mission_state_publish_period_sec").value)
            log_feedback_every = int(self.get_parameter("log_feedback_every_n_ticks").value)
            start_time = time.monotonic()

            # 主循环：轮询 goal 结果 + 安全态，直到进入终态
            while rclpy.ok() and context.state not in MissionStateMachine.TERMINAL_STATES:
                if goal_handle.is_cancel_requested:
                    self._process_event(
                        machine,
                        context,
                        navigator,
                        Event(EventType.ACTION_CANCEL_REQUESTED, "Mission canceled by action client."),
                    )
                    goal_handle.canceled()
                    break

                if context.state == MissionStateEnum.WAITING_FOR_RESULT:
                    safety_msg, _ = self._get_safety_snapshot()
                    # STOP_NOW 来自苏易的 safety_monitor
                    if safety_msg is not None and safety_msg.level == SafetyState.STOP_NOW:
                        self._process_event(
                            machine,
                            context,
                            navigator,
                            Event(
                                EventType.SAFETY_STOP,
                                safety_msg.reason or "Safety stop requested by safety manager.",
                            ),
                        )
                        continue

                    elapsed = time.monotonic() - (context.current_goal_start_monotonic or time.monotonic())
                    if elapsed > goal_timeout_sec:
                        self._process_event(
                            machine,
                            context,
                            navigator,
                            Event(EventType.GOAL_TIMEOUT, "Goal execution timed out."),
                        )
                        continue

                    if navigator.is_task_complete():
                        result = navigator.get_result()
                        event_type = {
                            NavTaskResult.SUCCEEDED: EventType.GOAL_SUCCEEDED,
                            NavTaskResult.CANCELED: EventType.GOAL_FAILED,
                            NavTaskResult.FAILED: EventType.GOAL_FAILED,
                            NavTaskResult.UNKNOWN: EventType.GOAL_FAILED,
                        }[result]
                        self._process_event(
                            machine,
                            context,
                            navigator,
                            Event(event_type, f"Goal finished with result {result.value}."),
                        )
                        continue

                    context.current_feedback = navigator.get_feedback()
                    feedback_counter += 1
                    if feedback_counter % max(log_feedback_every, 1) == 0 and context.current_feedback:
                        self.get_logger().info(
                            "[MISSION][WAITING_FOR_RESULT][M020] "
                            f"waypoint={context.current_index + 1}/{context.total_waypoints} "
                            f"distance_remaining={context.current_feedback.distance_remaining}"
                        )

                elif context.state == MissionStateEnum.PAUSED_FOR_SAFETY:
                    # 要求 SAFE 持续 safety_clear_hold_sec，防止抖动
                    if self._is_safety_clear_held(safety_hold_sec):
                        self._process_event(
                            machine,
                            context,
                            navigator,
                            Event(EventType.SAFETY_CLEAR, "Safety clear hold time satisfied."),
                        )
                        continue

                now = time.monotonic()
                if now - last_state_publish >= state_publish_period:
                    self._publish_action_feedback(goal_handle, context, start_time)
                    self._publish_state(context)
                    last_state_publish = now

                time.sleep(feedback_poll_period)

            if context.state == MissionStateEnum.MISSION_SUCCEEDED:
                goal_handle.succeed()
            elif context.state == MissionStateEnum.MISSION_CANCELED:
                goal_handle.canceled()
            elif context.state == MissionStateEnum.MISSION_FAILED:
                goal_handle.abort()

            return self._build_result(context, success=context.state == MissionStateEnum.MISSION_SUCCEEDED)
        finally:
            with self._goal_lock:
                self._active_goal = False

        # Action feedback 带进度百分比
    def _publish_action_feedback(
        self,
        goal_handle,
        context: MissionExecutionContext,
        start_time: float,
    ) -> None:
        feedback = RunMission.Feedback()
        feedback.current_waypoint_index = context.current_index
        feedback.current_state = context.state.value
        feedback.distance_remaining = (
            float(context.current_feedback.distance_remaining)
            if context.current_feedback and context.current_feedback.distance_remaining is not None
            else -1.0
        )
        feedback.elapsed_sec = float(time.monotonic() - start_time)
        goal_handle.publish_feedback(feedback)

    #         # 把 context 填进 RunMission.Result
        # 终态时组装 RunMission.Result
    def _build_result(self, context: MissionExecutionContext, success: bool) -> RunMission.Result:
        result = RunMission.Result()
        result.success = bool(success)
        result.completed_waypoints = context.completed_count
        result.failed_waypoints = context.failed_count
        result.final_state = context.state.value
        result.message = context.last_reason
        return result


# 入口：初始化 navigator、发 goal、主循环 spin
# mission_action_server 独立进程入口
def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionActionServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
