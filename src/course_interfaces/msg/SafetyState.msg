# ========================================================================
# 文件: src/course_interfaces/msg/SafetyState.msg
# 负责人: 苏易 | 需求: FR-D | PPT: 第19-20页 安全监控
# ========================================================================
#
# 【AI-PROMPT】
# 帮我定义 SafetyState.msg：level 枚举（SAFE/PAUSE/STOP_NOW 等）、reason、source 字段，给 mission_manager
# 和 safety_monitor 联调用，注释说明各 level 触发条件。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
string state_label
float32 min_range
string source
string reason
string detail
bool emergency_latched
uint32 recovery_attempts
