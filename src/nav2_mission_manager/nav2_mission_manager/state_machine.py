from .events import Event, EventType
from .models import MissionExecutionContext, StateTransition, TransitionCommand
from .retry_policy import FailureDecision, decide_after_failure
from .states import MissionState


class MissionStateMachine:
    """Explicit business-state transitions for mission execution."""

    TERMINAL_STATES = {
        MissionState.MISSION_SUCCEEDED,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    }

    def handle_event(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if context.state in self.TERMINAL_STATES:
            return StateTransition(context.state, reason=context.last_reason)

        handler_name = f"_handle_{context.state.value.lower()}"
        handler = getattr(self, handler_name, None)
        if handler is None:
            raise RuntimeError(f"No handler for state {context.state}.")
        transition = handler(context, event)
        context.state = transition.new_state
        if transition.reason:
            context.last_reason = transition.reason
        return transition

    def _handle_idle(self, context: MissionExecutionContext, event: Event) -> StateTransition:
        if event.type == EventType.MISSION_REQUESTED:
            return StateTransition(MissionState.LOADING_MISSION, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_loading_mission(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.MISSION_LOADED:
            return StateTransition(
                MissionState.WAITING_FOR_NAV2,
                command=TransitionCommand.WAIT_FOR_NAV2,
                reason=event.reason,
            )
        if event.type == EventType.MISSION_INVALID:
            return StateTransition(
                MissionState.MISSION_FAILED,
                command=TransitionCommand.FAIL_MISSION,
                reason=event.reason,
            )
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_waiting_for_nav2(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.NAV2_READY:
            return StateTransition(
                MissionState.DISPATCHING_GOAL,
                command=TransitionCommand.SEND_GOAL,
                reason=event.reason,
            )
        if event.type == EventType.NAV2_NOT_READY:
            return StateTransition(
                MissionState.MISSION_FAILED,
                command=TransitionCommand.FAIL_MISSION,
                reason=event.reason or "Nav2 did not become ready in time.",
            )
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_dispatching_goal(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.GOAL_SENT:
            return StateTransition(MissionState.WAITING_FOR_RESULT, reason=event.reason)
        if event.type == EventType.GOAL_REJECTED:
            return self._handle_goal_failure(context, event.reason or "Goal was rejected.")
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_waiting_for_result(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.GOAL_SUCCEEDED:
            context.completed_count += 1
            context.current_index += 1
            context.retry_count = 0
            context.current_feedback = None
            context.current_goal_start_monotonic = None
            if context.current_index >= context.total_waypoints:
                return StateTransition(
                    MissionState.MISSION_SUCCEEDED,
                    command=TransitionCommand.COMPLETE_MISSION,
                    reason=event.reason or "Mission completed successfully.",
                )
            return StateTransition(
                MissionState.DISPATCHING_GOAL,
                command=TransitionCommand.SEND_GOAL,
                reason=event.reason or "Advancing to the next waypoint.",
            )
        if event.type in {EventType.GOAL_FAILED, EventType.GOAL_TIMEOUT, EventType.GOAL_REJECTED}:
            return self._handle_goal_failure(context, event.reason)
        if event.type == EventType.SAFETY_STOP:
            context.safety_stop_count += 1
            context.current_feedback = None
            return StateTransition(
                MissionState.CANCELING_FOR_SAFETY,
                command=TransitionCommand.CANCEL_GOAL,
                reason=event.reason or "Safety stop requested.",
            )
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(
                MissionState.MISSION_CANCELED,
                command=TransitionCommand.CANCEL_GOAL,
                reason=event.reason or "Mission canceled by action client.",
            )
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_canceling_for_safety(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.ACTION_CANCEL_CONFIRMED:
            return StateTransition(
                MissionState.PAUSED_FOR_SAFETY,
                reason=event.reason or "Navigation canceled while waiting for safety clear.",
            )
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(
                MissionState.MISSION_CANCELED,
                reason=event.reason or "Mission canceled by action client.",
            )
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_paused_for_safety(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.SAFETY_CLEAR:
            return StateTransition(
                MissionState.DISPATCHING_GOAL,
                command=TransitionCommand.SEND_GOAL,
                reason=event.reason or "Safety clear received; re-dispatching current waypoint.",
            )
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_retrying_goal(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.GOAL_SENT:
            return StateTransition(MissionState.WAITING_FOR_RESULT, reason=event.reason)
        if event.type == EventType.GOAL_REJECTED:
            return self._handle_goal_failure(context, event.reason or "Retry dispatch rejected.")
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_skipping_goal(
        self,
        context: MissionExecutionContext,
        event: Event,
    ) -> StateTransition:
        if event.type == EventType.GOAL_SENT:
            return StateTransition(MissionState.WAITING_FOR_RESULT, reason=event.reason)
        if event.type == EventType.GOAL_REJECTED:
            return self._handle_goal_failure(context, event.reason or "Next goal dispatch rejected.")
        if event.type == EventType.ACTION_CANCEL_REQUESTED:
            return StateTransition(MissionState.MISSION_CANCELED, reason=event.reason)
        return StateTransition(context.state, reason=context.last_reason)

    def _handle_goal_failure(
        self,
        context: MissionExecutionContext,
        reason: str,
    ) -> StateTransition:
        context.retry_count += 1
        decision = decide_after_failure(
            retries_used=context.retry_count,
            max_retry_per_waypoint=context.max_retry_per_waypoint,
            allow_skip_waypoint=context.allow_skip_waypoint,
        )

        if decision == FailureDecision.RETRY:
            return StateTransition(
                MissionState.RETRYING_GOAL,
                command=TransitionCommand.SEND_GOAL,
                reason=reason or "Goal failed; retrying current waypoint.",
            )

        if decision == FailureDecision.SKIP:
            context.failed_count += 1
            context.current_index += 1
            context.retry_count = 0
            context.current_feedback = None
            context.current_goal_start_monotonic = None
            if context.current_index >= context.total_waypoints:
                return StateTransition(
                    MissionState.MISSION_SUCCEEDED,
                    command=TransitionCommand.COMPLETE_MISSION,
                    reason=reason or "Final waypoint skipped after retry exhaustion.",
                )
            return StateTransition(
                MissionState.SKIPPING_GOAL,
                command=TransitionCommand.SEND_GOAL,
                reason=reason or "Retry budget exhausted; skipping to the next waypoint.",
            )

        context.failed_count += 1
        context.current_feedback = None
        context.current_goal_start_monotonic = None
        return StateTransition(
            MissionState.MISSION_FAILED,
            command=TransitionCommand.FAIL_MISSION,
            reason=reason or "Retry budget exhausted; mission failed.",
        )
