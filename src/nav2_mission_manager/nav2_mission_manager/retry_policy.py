from enum import Enum


class FailureDecision(str, Enum):
    RETRY = "RETRY"
    SKIP = "SKIP"
    FAIL = "FAIL"


def should_retry(retries_used: int, max_retry_per_waypoint: int) -> bool:
    return retries_used <= max_retry_per_waypoint


def decide_after_failure(
    retries_used: int,
    max_retry_per_waypoint: int,
    allow_skip_waypoint: bool,
) -> FailureDecision:
    if should_retry(retries_used, max_retry_per_waypoint):
        return FailureDecision.RETRY
    if allow_skip_waypoint:
        return FailureDecision.SKIP
    return FailureDecision.FAIL
