from math import hypot, isfinite, radians
import time
from typing import Dict, Optional

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from course_interfaces.msg import SafetyState
from course_interfaces.srv import (
    GetSafetyState,
    RequestRecovery,
    TriggerCancel,
    TriggerEmergencyStop,
    TriggerPause,
)


STATE_NORMAL = "NORMAL"
STATE_PAUSED = "PAUSED"
STATE_EMERGENCY_STOP = "EMERGENCY_STOP"
STATE_CANCELED = "CANCELED"
STATE_RECOVERING = "RECOVERING"


class SafetyMonitor(Node):
    """Project side safety monitor for the Nav2 demo."""

    def __init__(self) -> None:
        super().__init__("safety_monitor")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("enable_sensor_monitor", True)
        self.declare_parameter("enable_tf_monitor", True)
        self.declare_parameter("enable_blockage_monitor", True)
        self.declare_parameter("enable_obstacle_monitor", False)
        self.declare_parameter("startup_grace_period_sec", 20.0)
        self.declare_parameter("tf_startup_grace_period_sec", 15.0)
        self.declare_parameter("tf_require_initial_transform", True)
        self.declare_parameter("sensor_timeout_sec", 10.0)
        self.declare_parameter("tf_timeout_sec", 10.0)
        self.declare_parameter("obstacle_pause_distance", 0.45)
        self.declare_parameter("obstacle_monitor_fov_deg", 60.0)
        self.declare_parameter("obstacle_monitor_requires_motion", True)
        self.declare_parameter("obstacle_monitor_motion_grace_sec", 1.0)
        self.declare_parameter("blockage_window_sec", 6.0)
        self.declare_parameter("min_progress_distance", 0.05)
        self.declare_parameter("min_cmd_vel_linear", 0.05)
        self.declare_parameter("min_cmd_vel_angular", 0.1)
        self.declare_parameter("blockage_cooldown_sec", 0.0)
        self.declare_parameter("auto_cancel_after_recovery_failures", 2)
        self.declare_parameter("recovery_grace_period_sec", 3.0)
        self.declare_parameter("monitor_period_sec", 0.2)
        self.declare_parameter("zero_cmd_publish_period_sec", 0.1)
        self.declare_parameter("recovery_clear_costmaps", True)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.enable_sensor_monitor = bool(
            self.get_parameter("enable_sensor_monitor").value
        )
        self.enable_tf_monitor = bool(self.get_parameter("enable_tf_monitor").value)
        self.enable_blockage_monitor = bool(
            self.get_parameter("enable_blockage_monitor").value
        )
        self.enable_obstacle_monitor = bool(
            self.get_parameter("enable_obstacle_monitor").value
        )
        self.startup_grace_period_sec = float(
            self.get_parameter("startup_grace_period_sec").value
        )
        self.tf_startup_grace_period_sec = float(
            self.get_parameter("tf_startup_grace_period_sec").value
        )
        self.tf_require_initial_transform = bool(
            self.get_parameter("tf_require_initial_transform").value
        )
        self.sensor_timeout_sec = float(self.get_parameter("sensor_timeout_sec").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.obstacle_pause_distance = float(
            self.get_parameter("obstacle_pause_distance").value
        )
        self.obstacle_monitor_fov_deg = float(
            self.get_parameter("obstacle_monitor_fov_deg").value
        )
        self.obstacle_monitor_requires_motion = bool(
            self.get_parameter("obstacle_monitor_requires_motion").value
        )
        self.obstacle_monitor_motion_grace_sec = float(
            self.get_parameter("obstacle_monitor_motion_grace_sec").value
        )
        self.blockage_window_sec = float(
            self.get_parameter("blockage_window_sec").value
        )
        self.min_progress_distance = float(
            self.get_parameter("min_progress_distance").value
        )
        self.min_cmd_vel_linear = float(
            self.get_parameter("min_cmd_vel_linear").value
        )
        self.min_cmd_vel_angular = float(
            self.get_parameter("min_cmd_vel_angular").value
        )
        self.blockage_cooldown_sec = float(
            self.get_parameter("blockage_cooldown_sec").value
        )
        self.auto_cancel_after_recovery_failures = int(
            self.get_parameter("auto_cancel_after_recovery_failures").value
        )
        self.recovery_grace_period_sec = float(
            self.get_parameter("recovery_grace_period_sec").value
        )
        self.recovery_clear_costmaps = bool(
            self.get_parameter("recovery_clear_costmaps").value
        )

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.state_pub = self.create_publisher(
            SafetyState, "/safety/state", state_qos
        )
        self.zero_cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_callback, 10)

        self.create_service(
            GetSafetyState, "/safety/get_safety_state", self._handle_get_state
        )
        self.create_service(
            TriggerPause, "/safety/trigger_pause", self._handle_trigger_pause
        )
        self.create_service(
            TriggerEmergencyStop,
            "/safety/trigger_emergency_stop",
            self._handle_trigger_estop,
        )
        self.create_service(
            TriggerCancel, "/safety/trigger_cancel", self._handle_trigger_cancel
        )
        self.create_service(
            RequestRecovery, "/safety/request_recovery", self._handle_recovery
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.cancel_goal_clients: Dict[str, CancelGoal] = {
            "/navigate_to_pose/_action/cancel_goal": self.create_client(
                CancelGoal, "/navigate_to_pose/_action/cancel_goal"
            ),
            "/follow_waypoints/_action/cancel_goal": self.create_client(
                CancelGoal, "/follow_waypoints/_action/cancel_goal"
            ),
            "/navigate_through_poses/_action/cancel_goal": self.create_client(
                CancelGoal, "/navigate_through_poses/_action/cancel_goal"
            ),
        }
        self.clear_costmap_clients: Dict[str, ClearEntireCostmap] = {
            "/local_costmap/clear_entirely_local_costmap": self.create_client(
                ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
            ),
            "/global_costmap/clear_entirely_global_costmap": self.create_client(
                ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
            ),
        }

        self.started_wall_time = time.monotonic()
        self.last_scan_wall_time = None
        self.latest_front_obstacle_distance = None
        self.last_tf_ok_wall_time = None
        self.has_seen_initial_tf = False
        self.last_odom_position = None
        self.blockage_reference_position = None
        self.nonzero_cmd_wall_since = None
        self.current_cmd_is_nonzero = False
        self.recovery_deadline_wall = None
        self.recovery_attempts = 0
        self.blockage_cooldown_deadline_wall = None

        self.current_state = STATE_NORMAL
        self.current_reason = "Safety monitor initialized"
        self.current_detail = ""
        self.current_source = "startup"
        self.last_state_change = self.get_clock().now()

        monitor_period = float(self.get_parameter("monitor_period_sec").value)
        zero_period = float(self.get_parameter("zero_cmd_publish_period_sec").value)
        self.monitor_timer = self.create_timer(monitor_period, self._monitor_loop)
        self.zero_cmd_timer = self.create_timer(zero_period, self._publish_zero_cmd)

        self._publish_state()
        self.get_logger().info("Safety monitor started")

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan_wall_time = time.monotonic()
        if self.enable_obstacle_monitor:
            self.latest_front_obstacle_distance = self._compute_front_obstacle_distance(
                msg
            )

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self.last_odom_position = (position.x, position.y)
        if self.blockage_reference_position is None:
            self.blockage_reference_position = self.last_odom_position

    def _cmd_vel_callback(self, msg: Twist) -> None:
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        now = time.monotonic()
        is_nonzero = (
            linear >= self.min_cmd_vel_linear or angular >= self.min_cmd_vel_angular
        )

        if is_nonzero:
            if not self.current_cmd_is_nonzero:
                self.nonzero_cmd_wall_since = now
                self.blockage_reference_position = self.last_odom_position
            self.current_cmd_is_nonzero = True
        else:
            self.current_cmd_is_nonzero = False
            self.nonzero_cmd_wall_since = None
            self.blockage_reference_position = self.last_odom_position

    def _handle_get_state(
        self, _: GetSafetyState.Request, response: GetSafetyState.Response
    ) -> GetSafetyState.Response:
        response.state = self._build_state_message()
        return response

    def _handle_trigger_pause(
        self, request: TriggerPause.Request, response: TriggerPause.Response
    ) -> TriggerPause.Response:
        response.success = self._transition_to_paused(
            request.reason or "Manual pause requested",
            source="manual",
            detail="Pause triggered via service",
        )
        response.state = self._build_state_message()
        return response

    def _handle_trigger_estop(
        self,
        request: TriggerEmergencyStop.Request,
        response: TriggerEmergencyStop.Response,
    ) -> TriggerEmergencyStop.Response:
        response.success = self._transition_to_estop(
            request.reason or "Manual emergency stop requested",
            source="manual",
            detail="Emergency stop triggered via service",
        )
        response.state = self._build_state_message()
        return response

    def _handle_trigger_cancel(
        self, request: TriggerCancel.Request, response: TriggerCancel.Response
    ) -> TriggerCancel.Response:
        response.success = self._transition_to_canceled(
            request.reason or "Manual cancel requested",
            source="manual",
            detail="Cancel triggered via service",
        )
        response.state = self._build_state_message()
        return response

    def _handle_recovery(
        self, request: RequestRecovery.Request, response: RequestRecovery.Response
    ) -> RequestRecovery.Response:
        strategy = request.strategy or "replan"
        reason = request.reason or "Manual recovery requested"
        response.accepted = self._start_recovery(
            strategy=strategy,
            reason=reason,
            source="manual",
            detail=f"Recovery strategy: {strategy}",
        )
        response.state = self._build_state_message()
        return response

    def _monitor_loop(self) -> None:
        now_wall = time.monotonic()

        self._check_tf(now_wall)

        if now_wall - self.started_wall_time < self.startup_grace_period_sec:
            return

        self._check_sensor_timeout(now_wall)
        self._check_obstacle_proximity(now_wall)
        self._check_blockage(now_wall)
        self._check_recovery_window(now_wall)

    def _compute_front_obstacle_distance(
        self, msg: LaserScan
    ) -> Optional[float]:
        if not msg.ranges:
            return None

        half_fov_rad = radians(self.obstacle_monitor_fov_deg) / 2.0
        best_distance = None

        for index, distance in enumerate(msg.ranges):
            if not isfinite(distance):
                continue
            if distance <= 0.0:
                continue
            if msg.range_min > 0.0 and distance < msg.range_min:
                continue
            if msg.range_max > 0.0 and distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) > half_fov_rad:
                continue

            if best_distance is None or distance < best_distance:
                best_distance = distance

        return best_distance

    def _check_sensor_timeout(self, now_wall: float) -> None:
        if not self.enable_sensor_monitor:
            return

        if self.sensor_timeout_sec <= 0:
            return

        if self.last_scan_wall_time is None:
            overdue = now_wall - self.started_wall_time
        else:
            overdue = now_wall - self.last_scan_wall_time

        if overdue > self.sensor_timeout_sec:
            self._transition_to_paused(
                reason="Sensor timeout detected",
                source="scan",
                detail=f"No LaserScan received on {self.scan_topic}",
            )

    def _check_obstacle_proximity(self, now_wall: float) -> None:
        if not self.enable_obstacle_monitor:
            return

        if self.obstacle_pause_distance <= 0.0:
            return

        if self.latest_front_obstacle_distance is None:
            return

        if self.obstacle_monitor_requires_motion:
            if not self.current_cmd_is_nonzero or self.nonzero_cmd_wall_since is None:
                return
            if (
                now_wall - self.nonzero_cmd_wall_since
                < self.obstacle_monitor_motion_grace_sec
            ):
                return

        if self.latest_front_obstacle_distance > self.obstacle_pause_distance:
            return

        self._transition_to_paused(
            reason="Obstacle proximity detected",
            source="scan_obstacle",
            detail=(
                "Front obstacle distance "
                f"{self.latest_front_obstacle_distance:.2f} m is below threshold "
                f"{self.obstacle_pause_distance:.2f} m"
            ),
        )

    def _check_tf(self, now_wall: float) -> None:
        if not self.enable_tf_monitor:
            return

        if self.tf_timeout_sec <= 0:
            return

        try:
            self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            self.last_tf_ok_wall_time = now_wall
            if not self.has_seen_initial_tf:
                self.has_seen_initial_tf = True
                self.get_logger().info(
                    f"Observed initial TF {self.odom_frame} -> {self.base_frame}; "
                    "strict TF timeout monitoring is now active"
                )
        except TransformException:
            if now_wall - self.started_wall_time < self.tf_startup_grace_period_sec:
                return

            if self.tf_require_initial_transform and not self.has_seen_initial_tf:
                return

            if self.last_tf_ok_wall_time is None:
                return

            overdue = now_wall - self.last_tf_ok_wall_time

            if overdue > self.tf_timeout_sec:
                self._transition_to_estop(
                    reason="TF timeout detected",
                    source="tf",
                    detail=f"Missing transform {self.odom_frame} -> {self.base_frame}",
                )

    def _check_blockage(self, now_wall: float) -> None:
        if not self.enable_blockage_monitor:
            return

        if self.blockage_window_sec <= 0:
            return

        if (
            self.blockage_cooldown_deadline_wall is not None
            and now_wall < self.blockage_cooldown_deadline_wall
        ):
            return

        if not self.current_cmd_is_nonzero or self.nonzero_cmd_wall_since is None:
            return

        if self.last_odom_position is None or self.blockage_reference_position is None:
            return

        if now_wall - self.nonzero_cmd_wall_since < self.blockage_window_sec:
            return

        progress = hypot(
            self.last_odom_position[0] - self.blockage_reference_position[0],
            self.last_odom_position[1] - self.blockage_reference_position[1],
        )
        if progress >= self.min_progress_distance:
            self.nonzero_cmd_wall_since = now_wall
            self.blockage_reference_position = self.last_odom_position
            return

        self.nonzero_cmd_wall_since = now_wall
        self.blockage_reference_position = self.last_odom_position
        self._start_recovery(
            strategy="replan",
            reason="Local blockage detected",
            source="blockage",
            detail="cmd_vel is active but odom progress stayed below threshold",
        )

    def _check_recovery_window(self, now_wall: float) -> None:
        if (
            self.current_state != STATE_RECOVERING
            or self.recovery_deadline_wall is None
        ):
            return

        if now_wall < self.recovery_deadline_wall:
            return

        if self._has_active_fault():
            return

        # Recovery attempts count consecutive failed/unstable recoveries.
        # Once we make it through the recovery grace period without a new fault,
        # treat that recovery as successful and clear the escalation counter.
        self.recovery_attempts = 0
        self._start_blockage_cooldown()
        self._set_state(
            STATE_NORMAL,
            reason="Recovery window completed",
            source="recovery",
            detail="No new safety fault detected during recovery grace period",
        )

    def _has_active_fault(self) -> bool:
        now_wall = time.monotonic()

        if self.enable_sensor_monitor and self.sensor_timeout_sec > 0:
            reference = self.last_scan_wall_time or self.started_wall_time
            if now_wall - reference > self.sensor_timeout_sec:
                return True

        if (
            self.enable_obstacle_monitor
            and self.obstacle_pause_distance > 0.0
            and self.latest_front_obstacle_distance is not None
            and self.latest_front_obstacle_distance <= self.obstacle_pause_distance
        ):
            return True

        if (
            self.enable_tf_monitor
            and self.tf_timeout_sec > 0
            and (not self.tf_require_initial_transform or self.has_seen_initial_tf)
        ):
            reference = self.last_tf_ok_wall_time
            if reference is not None and now_wall - reference > self.tf_timeout_sec:
                return True

        return False

    def _start_recovery(
        self, strategy: str, reason: str, source: str, detail: str
    ) -> bool:
        if self.current_state == STATE_CANCELED:
            self.get_logger().warn("Recovery request rejected because task is canceled")
            return False

        self.recovery_attempts += 1
        if self.recovery_attempts > self.auto_cancel_after_recovery_failures:
            return self._transition_to_canceled(
                reason="Recovery attempts exceeded threshold",
                source=source,
                detail=detail,
            )

        self._cancel_navigation_goals()
        if self.recovery_clear_costmaps:
            self._clear_costmaps()
        self.recovery_deadline_wall = (
            time.monotonic() + self.recovery_grace_period_sec
        )
        self._start_blockage_cooldown()
        return self._set_state(
            STATE_RECOVERING,
            reason=reason,
            source=source,
            detail=f"{detail}; strategy={strategy}",
        )

    def _transition_to_paused(self, reason: str, source: str, detail: str) -> bool:
        if self.current_state == STATE_CANCELED:
            return False
        if self.current_state == STATE_EMERGENCY_STOP and source != "manual":
            return False

        self._cancel_navigation_goals()
        return self._set_state(
            STATE_PAUSED, reason=reason, source=source, detail=detail
        )

    def _transition_to_estop(self, reason: str, source: str, detail: str) -> bool:
        if self.current_state == STATE_CANCELED:
            return False

        self._cancel_navigation_goals()
        self._publish_zero_cmd(force=True)
        return self._set_state(
            STATE_EMERGENCY_STOP, reason=reason, source=source, detail=detail
        )

    def _transition_to_canceled(self, reason: str, source: str, detail: str) -> bool:
        self._cancel_navigation_goals()
        self.recovery_deadline_wall = None
        return self._set_state(
            STATE_CANCELED, reason=reason, source=source, detail=detail
        )

    def _set_state(self, state: int, reason: str, source: str, detail: str) -> bool:
        changed = (
            self.current_state != state
            or self.current_reason != reason
            or self.current_detail != detail
            or self.current_source != source
        )
        self.current_state = state
        self.current_reason = reason
        self.current_detail = detail
        self.current_source = source
        self.last_state_change = self.get_clock().now()

        if state != STATE_RECOVERING:
            self.recovery_deadline_wall = None

        if changed:
            self._publish_state()
            self.get_logger().info(
                f"Safety state changed to {self._state_label(state)}: {reason}"
            )
        return True

    def _publish_state(self) -> None:
        self.state_pub.publish(self._build_state_message())

    def _build_state_message(self) -> SafetyState:
        msg = SafetyState()
        msg.level = self._state_level(self.current_state)
        msg.state_label = self._state_label(self.current_state)
        msg.reason = self.current_reason
        msg.detail = self.current_detail
        msg.source = self.current_source
        msg.stamp = self.last_state_change.to_msg()
        msg.min_range = float(self.latest_front_obstacle_distance or -1.0)
        msg.emergency_latched = self.current_state == STATE_EMERGENCY_STOP
        msg.recovery_attempts = self.recovery_attempts
        return msg

    def _state_label(self, state: str) -> str:
        return state

    def _state_level(self, state: str) -> int:
        if state == STATE_NORMAL:
            return SafetyState.SAFE
        if state == STATE_RECOVERING:
            return SafetyState.SLOW_DOWN
        return SafetyState.STOP_NOW

    def _cancel_navigation_goals(self) -> None:
        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        for service_name, client in self.cancel_goal_clients.items():
            if not client.wait_for_service(timeout_sec=0.05):
                continue
            future = client.call_async(request)
            future.add_done_callback(
                lambda finished, service=service_name: self._log_cancel_result(
                    service, finished
                )
            )

    def _log_cancel_result(self, service_name: str, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging
            self.get_logger().warn(
                f"Failed to cancel goals via {service_name}: {exc}"
            )
            return

        if result.return_code == CancelGoal.Response.ERROR_NONE:
            self.get_logger().info(f"Cancel request accepted on {service_name}")

    def _clear_costmaps(self) -> None:
        request = ClearEntireCostmap.Request()
        for service_name, client in self.clear_costmap_clients.items():
            if not client.wait_for_service(timeout_sec=0.05):
                continue
            future = client.call_async(request)
            future.add_done_callback(
                lambda _, service=service_name: self.get_logger().info(
                    f"Requested costmap clear on {service}"
                )
            )

    def _publish_zero_cmd(self, force: bool = False) -> None:
        if not force and self.current_state != STATE_EMERGENCY_STOP:
            return

        self.zero_cmd_pub.publish(Twist())

    def _start_blockage_cooldown(self) -> None:
        if self.blockage_cooldown_sec <= 0:
            self.blockage_cooldown_deadline_wall = None
            return

        self.blockage_cooldown_deadline_wall = (
            time.monotonic() + self.blockage_cooldown_sec
        )
        self.nonzero_cmd_wall_since = None
        self.blockage_reference_position = self.last_odom_position


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
