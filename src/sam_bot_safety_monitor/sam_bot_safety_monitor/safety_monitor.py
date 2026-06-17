# ========================================================================
# 文件: src/sam_bot_safety_monitor/sam_bot_safety_monitor/safety_monitor.py
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# SafetyMonitor 主循环：定时检查 scan 超时、TF 超时、前方障碍距离、局部堵塞（cmd_vel vs odom），状态变化时发布 SafetyState 并
# zero cmd_vel。请生成 _monitor_tick 和各 _check_* 方法声明框架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ---------------------------------------------------------------------------
# 【实现说明】
# 定时器 _monitor_loop 按顺序检查 TF、scan 超时、近障、局部堵塞；
# 状态变化时发布 /safety/state（TRANSIENT_LOCAL），estop 时额外 zero cmd_vel。
# 与 mission_manager 联调：PAUSED/ESTOP 映射为 STOP_NOW，RECOVERING 为 SLOW_DOWN。
# ---------------------------------------------------------------------------

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

# 字符串状态主要给日志和 demo 脚本看，真正对外发布的是 SafetyState.level


# TODO[苏易]：FR-D-01~05 安全监控主节点，传感器/TF/障碍/堵塞检测
class SafetyMonitor(Node):
    """Project side safety monitor for the Nav2 demo."""

    def __init__(self) -> None:
        super().__init__("safety_monitor")

        # 话题和坐标系名字都做成参数，方便换仿真/真机
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        # 可单独关 scan 超时，调 demo 时方便
        # --- 传感器监控 ---
        self.declare_parameter("enable_sensor_monitor", True)
        self.declare_parameter("enable_tf_monitor", True)
        self.declare_parameter("enable_blockage_monitor", True)
        # 近障 pause 默认关，reach_goal demo 再开
        self.declare_parameter("enable_obstacle_monitor", False)
        # Gazebo 刚起来先别判超时
        # --- 启动宽限期 ---
        self.declare_parameter("startup_grace_period_sec", 20.0)
        self.declare_parameter("tf_startup_grace_period_sec", 15.0)
        self.declare_parameter("tf_require_initial_transform", True)
        # --- 超时阈值 ---
        self.declare_parameter("sensor_timeout_sec", 10.0)
        self.declare_parameter("tf_timeout_sec", 10.0)
        # --- 近障检测（可选）---
        self.declare_parameter("obstacle_pause_distance", 0.45)
        self.declare_parameter("obstacle_monitor_fov_deg", 60.0)
        self.declare_parameter("obstacle_monitor_requires_motion", True)
        self.declare_parameter("obstacle_monitor_motion_grace_sec", 1.0)
        # 有 cmd_vel 但 odom 不动超过这个窗口算堵塞
        # --- 局部堵塞检测 ---
        self.declare_parameter("blockage_window_sec", 6.0)
        self.declare_parameter("min_progress_distance", 0.05)
        self.declare_parameter("min_cmd_vel_linear", 0.05)
        self.declare_parameter("min_cmd_vel_angular", 0.1)
        self.declare_parameter("blockage_ignore_during_spin", True)
        self.declare_parameter("blockage_cooldown_sec", 0.0)
        # --- recovery 策略 ---
        self.declare_parameter("auto_cancel_after_recovery_failures", 2)
        self.declare_parameter("recovery_grace_period_sec", 3.0)
        # --- 定时器 ---
        self.declare_parameter("monitor_period_sec", 0.2)
        self.declare_parameter("zero_cmd_publish_period_sec", 0.1)
        self.declare_parameter("recovery_clear_costmaps", True)

        # 激光话题名
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        # 速度输出话题，estop 时 zero
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        # 里程计 frame
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
        self.blockage_ignore_during_spin = bool(
            self.get_parameter("blockage_ignore_during_spin").value
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

        # TRANSIENT_LOCAL：后启动的 mission_manager 也能拿到最新安全态
        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.state_pub = self.create_publisher(
            SafetyState, "/safety/state", state_qos
        )
        self.zero_cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # 三个输入：激光、里程计、当前速度指令
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

        # Nav2 可能走 to_pose / waypoints / through_poses，三个 cancel 客户端都备着
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
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.recovery_deadline_wall = None
        self.recovery_attempts = 0
        self.blockage_cooldown_deadline_wall = None

        # 运行时状态，_set_state 里统一维护
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

    # 激光回调：刷新时间戳，可选算前方最近障碍
    def _scan_callback(self, msg: LaserScan) -> None:
        # 只要收到 scan 就刷新时间戳，超时检测在定时器里做
        self.last_scan_wall_time = time.monotonic()
        if self.enable_obstacle_monitor:
            self.latest_front_obstacle_distance = self._compute_front_obstacle_distance(
                msg
            )

    # 里程计回调：记录位移，供堵塞检测用
    def _odom_callback(self, msg: Odometry) -> None:
        # 用 odom 位移判断局部堵塞：有速度指令但几乎不动
        position = msg.pose.pose.position
        self.last_odom_position = (position.x, position.y)
        if self.blockage_reference_position is None:
            self.blockage_reference_position = self.last_odom_position

    # 速度指令回调：判断是否在执行 nonzero cmd
    def _cmd_vel_callback(self, msg: Twist) -> None:
        # 记录什么时候开始“真的在走”，堵塞检测需要这段窗口
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        self.last_cmd_linear = linear
        self.last_cmd_angular = angular
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

    # 服务查询当前 SafetyState，RViz 面板也会用
    def _handle_get_state(
        self, _: GetSafetyState.Request, response: GetSafetyState.Response
    ) -> GetSafetyState.Response:
        response.state = self._build_state_message()
        return response

    # 手动 pause，期末演示 injector 会调
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

    # 急停：cancel goal + 持续 zero cmd_vel
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

    # 整任务取消，recoverable 置 false
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

    # 外部请求 recovery，走清 costmap + replan
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

    # 定时巡检入口，grace period 内只做 TF 检查
    def _monitor_loop(self) -> None:
        now_wall = time.monotonic()

        self._check_tf(now_wall)

        # Gazebo 刚起来时 scan/tf 都不稳，先给一段 grace period
        if now_wall - self.started_wall_time < self.startup_grace_period_sec:
            return

        self._check_sensor_timeout(now_wall)
        self._check_obstacle_proximity(now_wall)
        self._check_blockage(now_wall)
        self._check_recovery_window(now_wall)
        self._check_pause_clear()

    # 扫描前方 FOV 内最近有效 range
    def _compute_front_obstacle_distance(
        self, msg: LaserScan
    ) -> Optional[float]:
        # 只关心车体前方 FOV 内的最近障碍，侧后方忽略
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

    # scan 超时 -> pause
    def _check_sensor_timeout(self, now_wall: float) -> None:
        if not self.enable_sensor_monitor:
            return

        if self.sensor_timeout_sec <= 0:
            return

        # 从未收到 scan 时，从节点启动时刻算起
        if self.last_scan_wall_time is None:
            overdue = now_wall - self.started_wall_time
        else:
            overdue = now_wall - self.last_scan_wall_time

        if overdue > self.sensor_timeout_sec:
            # 传感器挂了先 pause，别直接 estop，给恢复留余地
            self._transition_to_paused(
                reason="Sensor timeout detected",
                source="scan",
                detail=f"No LaserScan received on {self.scan_topic}",
            )

    # 前方距离低于阈值 -> pause
    def _check_obstacle_proximity(self, now_wall: float) -> None:
        if not self.enable_obstacle_monitor:
            return

        if self.obstacle_pause_distance <= 0.0:
            return

        if self.latest_front_obstacle_distance is None:
            return

        if self.obstacle_monitor_requires_motion:
            # 静止时不判近障，避免 startup 时 scan 贴墙误报
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

    # odom->base_link TF 超时 -> estop
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
            # TF 树还没建全时别急着报错
            if now_wall - self.started_wall_time < self.tf_startup_grace_period_sec:
                return

            if self.tf_require_initial_transform and not self.has_seen_initial_tf:
                return

            if self.last_tf_ok_wall_time is None:
                return

            overdue = now_wall - self.last_tf_ok_wall_time

            if overdue > self.tf_timeout_sec:
                # TF 断了比 scan 超时更严重，直接 estop
                self._transition_to_estop(
                    reason="TF timeout detected",
                    source="tf",
                    detail=f"Missing transform {self.odom_frame} -> {self.base_frame}",
                )

    def _is_spinning_in_place(self) -> bool:
        """Nav2 spin recovery: angular cmd dominates, little linear motion."""
        return (
            self.last_cmd_angular >= self.min_cmd_vel_angular
            and self.last_cmd_linear < self.min_cmd_vel_linear
        )

    # cmd_vel 有但 odom 几乎不动 -> recovery
    def _check_blockage(self, now_wall: float) -> None:
        if not self.enable_blockage_monitor:
            return

        if self.blockage_window_sec <= 0:
            return

        if self.blockage_ignore_during_spin and self._is_spinning_in_place():
            # Spin 脱困时 odom 位移小是正常的，不要 cancel Nav2 recovery
            self.nonzero_cmd_wall_since = now_wall
            self.blockage_reference_position = self.last_odom_position
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
            # 其实有在动，重置堵塞观察窗口
            self.nonzero_cmd_wall_since = now_wall
            self.blockage_reference_position = self.last_odom_position
            return

        # 有速度指令但位移不够：尝试 recovery（清 costmap + replan）
        self.nonzero_cmd_wall_since = now_wall
        self.blockage_reference_position = self.last_odom_position
        self._start_recovery(
            strategy="replan",
            reason="Local blockage detected",
            source="blockage",
            detail="cmd_vel is active but odom progress stayed below threshold",
        )

    # recovery 观察期结束且无新故障 -> NORMAL
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

        # 恢复观察期里没再触发故障，就算这次 recovery 成功
        self.recovery_attempts = 0
        self._start_blockage_cooldown()
        self._set_state(
            STATE_NORMAL,
            reason="Recovery window completed",
            source="recovery",
            detail="No new safety fault detected during recovery grace period",
        )

    def _check_pause_clear(self) -> None:
        if self.current_state != STATE_PAUSED:
            return
        if self.current_source == "manual":
            return
        if self._has_active_fault():
            return

        self.recovery_attempts = 0
        self._start_blockage_cooldown()
        self._set_state(
            STATE_NORMAL,
            reason="Safety pause cleared",
            source="safety_clear",
            detail="No active safety fault remains after backing away",
        )

        # recovery 窗口结束时检查故障是否还在
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

    # cancel Nav2 + 可选清 costmap，进入 RECOVERING
    def _start_recovery(
        self, strategy: str, reason: str, source: str, detail: str
    ) -> bool:
        if self.current_state == STATE_CANCELED:
            self.get_logger().warn("Recovery request rejected because task is canceled")
            return False

        self.recovery_attempts += 1
        # 连续 recovery 太多次就放弃，避免无限 replan 循环
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

    # 进入 PAUSED 并 cancel 导航 goal
    def _transition_to_paused(self, reason: str, source: str, detail: str) -> bool:
        if self.current_state == STATE_CANCELED:
            return False
        # 已经 estop 了就不要被 scan 超时降级成 pause
        if self.current_state == STATE_EMERGENCY_STOP and source != "manual":
            return False

        # LINK[徐梓鸣]：pause 时 mission_manager 会收到 STOP_NOW
        self._cancel_navigation_goals()
        return self._set_state(
            STATE_PAUSED, reason=reason, source=source, detail=detail
        )

    # 急停：cancel + 持续 zero cmd_vel
    def _transition_to_estop(self, reason: str, source: str, detail: str) -> bool:
        if self.current_state == STATE_CANCELED:
            return False

        self._cancel_navigation_goals()
        self._publish_zero_cmd(force=True)
        return self._set_state(
            STATE_EMERGENCY_STOP, reason=reason, source=source, detail=detail
        )

    # 任务取消，不再接受 recovery
    def _transition_to_canceled(self, reason: str, source: str, detail: str) -> bool:
        self._cancel_navigation_goals()
        self.recovery_deadline_wall = None
        return self._set_state(
            STATE_CANCELED, reason=reason, source=source, detail=detail
        )

        # 统一改状态并发布 /safety/state
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

    # 发布 SafetyState 话题
    def _publish_state(self) -> None:
        self.state_pub.publish(self._build_state_message())

        # 填 min_range 等字段，mission 侧读 level
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

    # 内部字符串标签，与 msg.state_label 一致
    def _state_label(self, state: str) -> str:
        return state

    # 映射到 SAFE / SLOW_DOWN / STOP_NOW
    def _state_level(self, state: str) -> int:
        # 映射到 course_interfaces/SafetyState，mission 侧只看 level
        if state == STATE_NORMAL:
            return SafetyState.SAFE
        if state == STATE_RECOVERING:
            return SafetyState.SLOW_DOWN
        return SafetyState.STOP_NOW

    # 异步 cancel 三个 Nav2 action
    def _cancel_navigation_goals(self) -> None:
        # 异步 cancel，不阻塞 monitor 定时器
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

    # cancel 完成回调，打日志
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

    # recovery 时清 local/global costmap
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

    # estop 期间周期性发零速度
    def _publish_zero_cmd(self, force: bool = False) -> None:
        # estop 时持续发零速度，防止控制器还在输出
        if not force and self.current_state != STATE_EMERGENCY_STOP:
            return

        self.zero_cmd_pub.publish(Twist())

    # 堵塞触发后冷却，避免立刻再判
    def _start_blockage_cooldown(self) -> None:
        if self.blockage_cooldown_sec <= 0:
            self.blockage_cooldown_deadline_wall = None
            return

        self.blockage_cooldown_deadline_wall = (
            time.monotonic() + self.blockage_cooldown_sec
        )
        self.nonzero_cmd_wall_since = None
        self.blockage_reference_position = self.last_odom_position


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
