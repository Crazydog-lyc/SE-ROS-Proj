import copy
import time
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from course_interfaces.msg import SafetyState


STATE_NORMAL = "NORMAL"
STATE_PAUSED = "PAUSED"
STATE_EMERGENCY_STOP = "EMERGENCY_STOP"
STATE_CANCELED = "CANCELED"
STATE_RECOVERING = "RECOVERING"


class SafetyAwareNavigator(BasicNavigator):
    """Thin project-side navigator wrapper that reacts to safety state."""

    def __init__(
        self,
        node_name: str = "safety_aware_navigator",
        recovery_mode: str = "resume_remaining",
    ) -> None:
        super().__init__(node_name=node_name)

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.declare_parameter("recovery_mode", recovery_mode)
        self._recovery_mode = str(self.get_parameter("recovery_mode").value)
        if self._recovery_mode not in (
            "resume_remaining",
            "restart_full_mission",
            "client_replay_after_normal",
        ):
            self.warn(
                f"Unsupported recovery mode '{self._recovery_mode}', "
                "falling back to resume_remaining"
            )
            self._recovery_mode = "resume_remaining"

        self.safety_level = SafetyState.SAFE
        self.safety_state_label = STATE_NORMAL
        self.safety_reason = ""
        self.safety_source = "startup"
        self._saved_task_type: Optional[str] = None
        self._saved_pose: Optional[PoseStamped] = None
        self._saved_waypoints: List[PoseStamped] = []
        self._original_pose: Optional[PoseStamped] = None
        self._original_waypoints: List[PoseStamped] = []
        self._mission_start_pose: Optional[PoseStamped] = None
        self._saved_behavior_tree = ""
        self._pending_safety_cancel = False
        self._pending_recovery_resume = False
        self._recoverable = False
        self._task_active = False
        self._last_cancellation_was_safety = False
        self._current_waypoint_index = 0
        self._waypoint_feedback_offset = 0
        self._recovery_phase: Optional[str] = None
        self._cancel_grace_deadline = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )

        self.create_subscription(
            SafetyState, "/safety/state", self._on_safety_state, state_qos
        )

    def _on_safety_state(self, msg: SafetyState) -> None:
        state_changed = (
            self.safety_state_label != msg.state_label
            or self.safety_level != msg.level
        )
        self.safety_level = msg.level
        self.safety_state_label = msg.state_label
        self.safety_reason = msg.reason
        self.safety_source = msg.source

        if state_changed:
            self.info(f"Safety state changed to {msg.state_label}: {msg.reason}")

        if msg.state_label in (STATE_PAUSED, STATE_EMERGENCY_STOP, STATE_CANCELED):
            self._pending_safety_cancel = True
            if msg.state_label == STATE_CANCELED:
                self._recoverable = False
        elif msg.state_label == STATE_RECOVERING and self._recoverable:
            self._pending_recovery_resume = True

    def _feedbackCallback(self, msg) -> None:
        super()._feedbackCallback(msg)
        feedback = msg.feedback
        if self._saved_task_type == "waypoints" and hasattr(feedback, "current_waypoint"):
            self._current_waypoint_index = (
                self._waypoint_feedback_offset + int(feedback.current_waypoint)
            )

    def goToPose(self, pose, behavior_tree=""):
        if not self._can_start_new_task():
            return False

        self._mission_start_pose = self._ensure_mission_start_pose()
        success = super().goToPose(pose, behavior_tree)
        if success:
            if self._mission_start_pose is None:
                self._mission_start_pose = self._ensure_mission_start_pose()
            self._saved_task_type = "pose"
            self._saved_pose = copy.deepcopy(pose)
            self._saved_waypoints = []
            self._original_pose = copy.deepcopy(pose)
            self._original_waypoints = []
            self._saved_behavior_tree = behavior_tree
            self._recoverable = True
            self._task_active = True
            self._last_cancellation_was_safety = False
            self._recovery_phase = None
            self._cancel_grace_deadline = None
        return success

    def followWaypoints(self, poses):
        if not self._can_start_new_task():
            return False

        self._mission_start_pose = self._ensure_mission_start_pose()
        success = super().followWaypoints(poses)
        if success:
            if self._mission_start_pose is None:
                self._mission_start_pose = self._ensure_mission_start_pose()
            self._saved_task_type = "waypoints"
            self._saved_waypoints = [copy.deepcopy(pose) for pose in poses]
            self._saved_pose = None
            self._original_pose = None
            self._original_waypoints = [copy.deepcopy(pose) for pose in poses]
            self._saved_behavior_tree = ""
            self._recoverable = True
            self._task_active = True
            self._current_waypoint_index = 0
            self._waypoint_feedback_offset = 0
            self._last_cancellation_was_safety = False
            self._recovery_phase = None
            self._cancel_grace_deadline = None
        return success

    def isTaskComplete(self):
        self._process_safety_events()

        if self.is_waiting_for_recovery():
            return False

        complete = super().isTaskComplete()
        if (
            complete
            and self.status == GoalStatus.STATUS_SUCCEEDED
            and self._recovery_phase is not None
        ):
            if self._continue_restarted_mission():
                return False

        if complete and self.status == GoalStatus.STATUS_CANCELED and self._recoverable:
            if self.safety_state_label == STATE_CANCELED:
                self._task_active = False
                self._recoverable = False
                return True

            # A safety-triggered cancel from an earlier task can arrive after we
            # have already re-submitted the mission. Treat it as transient and
            # keep the client alive instead of exiting the demo early.
            if self.safety_state_label in (
                STATE_PAUSED,
                STATE_EMERGENCY_STOP,
                STATE_RECOVERING,
                STATE_NORMAL,
            ):
                if self._cancel_grace_deadline is None:
                    self._cancel_grace_deadline = (
                        self.get_clock().now() + Duration(seconds=1.0)
                    )
                    return False

                if self.get_clock().now() < self._cancel_grace_deadline:
                    return False

                return False

        if complete:
            self._task_active = False
            if self.safety_state_label == STATE_CANCELED:
                self._recoverable = False
            elif self.status == GoalStatus.STATUS_SUCCEEDED:
                self._clear_saved_context()
            self._cancel_grace_deadline = None
        return complete

    def _process_safety_events(self) -> None:
        if self._pending_safety_cancel:
            self._pending_safety_cancel = False
            if self._task_active and self.result_future:
                self.info("Safety canceled current task")
                super().cancelTask()
                self._task_active = False
                self._last_cancellation_was_safety = True
                self._cancel_grace_deadline = (
                    self.get_clock().now() + Duration(seconds=1.0)
                )

        if self._pending_recovery_resume:
            self._pending_recovery_resume = False
            if not self._recoverable:
                return
            if self.safety_state_label != STATE_RECOVERING:
                return
            if self._recovery_mode == "client_replay_after_normal":
                self.info(
                    "Safety recovery requested, deferring replay to client after NORMAL"
                )
                return
            self._resume_saved_task()

    def _resume_saved_task(self) -> None:
        if self._saved_task_type == "pose" and self._saved_pose is not None:
            self.info(f"Recovery mode is {self._recovery_mode}")
            if (
                self._recovery_mode == "restart_full_mission"
                and self._mission_start_pose is not None
                and self._original_pose is not None
            ):
                self.info(
                    "Safety recovery requested, returning to mission start pose "
                    "before replaying original goal"
                )
                if super().goToPose(copy.deepcopy(self._mission_start_pose)):
                    self._task_active = True
                    self._last_cancellation_was_safety = False
                    self._cancel_grace_deadline = None
                    self._recovery_phase = "return_to_start_for_pose"
                    self.info("Navigation resumed")
                return
            if self._recovery_mode == "restart_full_mission":
                self.warn(
                    "Mission start pose is unavailable, falling back to resubmitting "
                    "the saved goal"
                )

            self.info("Safety recovery requested, resubmitting goal")
            if super().goToPose(copy.deepcopy(self._saved_pose), self._saved_behavior_tree):
                self._task_active = True
                self._last_cancellation_was_safety = False
                self._cancel_grace_deadline = None
                self._recovery_phase = None
                self.info("Navigation resumed")
            return

        if self._saved_task_type == "waypoints" and self._saved_waypoints:
            self.info(f"Recovery mode is {self._recovery_mode}")
            if (
                self._recovery_mode == "restart_full_mission"
                and self._mission_start_pose is not None
                and self._original_waypoints
            ):
                self.info(
                    "Safety recovery requested, returning to mission start pose "
                    "before replaying full mission"
                )
                if super().goToPose(copy.deepcopy(self._mission_start_pose)):
                    self._task_active = True
                    self._last_cancellation_was_safety = False
                    self._current_waypoint_index = 0
                    self._waypoint_feedback_offset = 0
                    self._cancel_grace_deadline = None
                    self._recovery_phase = "return_to_start_for_waypoints"
                    self.info("Navigation resumed")
                return
            if self._recovery_mode == "restart_full_mission":
                self.warn(
                    "Mission start pose is unavailable, falling back to remaining "
                    "waypoints"
                )

            resume_start_index = self._current_waypoint_index
            remaining_waypoints = self._saved_waypoints[resume_start_index:]
            if not remaining_waypoints:
                return
            self.info("Safety recovery requested, resubmitting remaining waypoints")
            if super().followWaypoints(
                [copy.deepcopy(pose) for pose in remaining_waypoints]
            ):
                self._task_active = True
                self._last_cancellation_was_safety = False
                self._waypoint_feedback_offset = resume_start_index
                self._cancel_grace_deadline = None
                self._recovery_phase = None
                self.info("Navigation resumed")

    def _continue_restarted_mission(self) -> bool:
        if self._recovery_phase == "return_to_start_for_pose" and self._original_pose is not None:
            self.info("Reached mission start pose, replaying original goal")
            if super().goToPose(
                copy.deepcopy(self._original_pose), self._saved_behavior_tree
            ):
                self._task_active = True
                self._last_cancellation_was_safety = False
                self._cancel_grace_deadline = None
                self._recovery_phase = None
                self.info("Navigation resumed")
                return True
            self.warn("Failed to replay original goal after returning to mission start pose")
            self._recovery_phase = None
            return False

        if (
            self._recovery_phase == "return_to_start_for_waypoints"
            and self._original_waypoints
        ):
            self.info("Reached mission start pose, replaying full mission")
            if super().followWaypoints(
                [copy.deepcopy(pose) for pose in self._original_waypoints]
            ):
                self._task_active = True
                self._last_cancellation_was_safety = False
                self._current_waypoint_index = 0
                self._waypoint_feedback_offset = 0
                self._cancel_grace_deadline = None
                self._recovery_phase = None
                self.info("Navigation resumed")
                return True
            self.warn(
                "Failed to replay original waypoint mission after returning to mission start pose"
            )
            self._recovery_phase = None

        return False

    def _capture_current_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.warn(f"Unable to capture mission start pose from TF: {exc}")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        return pose

    def _ensure_mission_start_pose(self) -> Optional[PoseStamped]:
        pose = self._capture_current_pose()
        if pose is not None:
            self.info("Captured mission start pose")
            return pose

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self._capture_current_pose()
            if pose is not None:
                self.info("Captured mission start pose after waiting for TF")
                return pose

        self.warn(
            "Unable to capture mission start pose before starting navigation task"
        )
        return None

    def _can_start_new_task(self) -> bool:
        if self.safety_state_label != STATE_NORMAL:
            self.warn(
                f"Refusing to start navigation while safety state is "
                f"{self.safety_state_label}"
            )
            return False
        return True

    def _clear_saved_context(self) -> None:
        self._saved_task_type = None
        self._saved_pose = None
        self._saved_waypoints = []
        self._original_pose = None
        self._original_waypoints = []
        self._mission_start_pose = None
        self._saved_behavior_tree = ""
        self._recoverable = False
        self._current_waypoint_index = 0
        self._waypoint_feedback_offset = 0
        self._recovery_phase = None
        self._cancel_grace_deadline = None

    def get_safety_state_label(self) -> str:
        return self.safety_state_label

    def get_safety_reason(self) -> str:
        return self.safety_reason

    def get_safety_source(self) -> str:
        return self.safety_source

    def is_waiting_for_recovery(self) -> bool:
        return self._recoverable and self.safety_state_label in (
            STATE_PAUSED,
            STATE_EMERGENCY_STOP,
            STATE_RECOVERING,
        )

    def is_terminal_canceled(self) -> bool:
        return self.safety_state_label == STATE_CANCELED

    def get_current_waypoint_index(self) -> int:
        return self._current_waypoint_index

    def get_recovery_mode(self) -> str:
        return self._recovery_mode
