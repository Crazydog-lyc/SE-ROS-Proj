from nav2_mission_manager.retry_policy import FailureDecision, decide_after_failure, should_retry


def test_should_retry_within_budget():
    assert should_retry(1, 1) is True


def test_should_retry_exhausted_budget():
    assert should_retry(2, 1) is False


def test_decide_after_failure_retry_then_skip_then_fail():
    assert decide_after_failure(1, 1, False) == FailureDecision.RETRY
    assert decide_after_failure(2, 1, True) == FailureDecision.SKIP
    assert decide_after_failure(2, 1, False) == FailureDecision.FAIL
