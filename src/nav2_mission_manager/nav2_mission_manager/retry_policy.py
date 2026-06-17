# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/retry_policy.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# decide_after_failure(retries_used, max_retry, allow_skip) 返回 RETRY/SKIP/FAIL
# 枚举，逻辑简单：未超重试次数则 RETRY，否则 allow_skip 则 SKIP 否则 FAIL。请生成函数骨架。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
from enum import Enum



class FailureDecision(str, Enum):
    RETRY = "RETRY"
    SKIP = "SKIP"
    FAIL = "FAIL"


def should_retry(retries_used: int, max_retry_per_waypoint: int) -> bool:
    # retries_used 从 1 开始计，和 Action goal 里的 max_retry 对齐
    return retries_used <= max_retry_per_waypoint


def decide_after_failure(
    retries_used: int,
    max_retry_per_waypoint: int,
    allow_skip_waypoint: bool,
) -> FailureDecision:
    # 先重试，再跳过，最后整任务失败——顺序写死在策略里
    if should_retry(retries_used, max_retry_per_waypoint):
        return FailureDecision.RETRY
    if allow_skip_waypoint:
        return FailureDecision.SKIP
    return FailureDecision.FAIL
