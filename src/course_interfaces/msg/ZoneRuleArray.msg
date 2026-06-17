# ========================================================================
# 文件: src/course_interfaces/msg/ZoneRuleArray.msg
# 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
# ========================================================================
#
# 【AI-PROMPT】
# 帮我定义 ZoneRule/ZoneRuleArray.msg：描述矩形/圆/多边形语义区域、cost、mode，供 semantic costmap layer
# 加载，字段和注释按 Nav2 costmap 习惯来写。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
# [src/course_interfaces/msg/ZoneRuleArray.msg] 负责人: 李熠城
# 【AI】提示词: "帮我定义 ZoneRule/ZoneRuleArray.msg：描述矩形/圆/多边形语义区域、cost、mode，供 semantic costmap layer 加载，字段和注释按 Nav2 costmap 习惯来写。"
# TODO[李熠城]：FR-C 接口字段定义
ZoneRule[] rules
