# ========================================================================
# 文件: src/nav2_mission_manager/test/test_retry_policy.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# decide_after_failure(retries_used, max_retry, allow_skip) 返回 RETRY/SKIP/FAIL
# 枚举，逻辑简单：未超重试次数则 RETRY，否则 allow_skip 则 SKIP 否则 FAIL。请生成函数骨架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from nav2_mission_manager.retry_policy import FailureDecision, decide_after_failure, should_retry



# 单测：def should_retry_within_budget
def test_should_retry_within_budget():
    assert should_retry(1, 1) is True


# 单测：def should_retry_exhausted_budget
def test_should_retry_exhausted_budget():
    assert should_retry(2, 1) is False


# 单测：def decide_after_failure_retry_then_skip_then_fail
def test_decide_after_failure_retry_then_skip_then_fail():
    assert decide_after_failure(1, 1, False) == FailureDecision.RETRY
    assert decide_after_failure(2, 1, True) == FailureDecision.SKIP
    assert decide_after_failure(2, 1, False) == FailureDecision.FAIL
