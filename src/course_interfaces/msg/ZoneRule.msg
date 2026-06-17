# ========================================================================
# 文件: src/course_interfaces/msg/ZoneRule.msg
# 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
# ========================================================================
#
# 【AI-PROMPT】
# 帮我定义 ZoneRule/ZoneRuleArray.msg：描述矩形/圆/多边形语义区域、cost、mode，供 semantic costmap layer
# 加载，字段和注释按 Nav2 costmap 习惯来写。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
string zone_id
uint8 KEEP_OUT=0
uint8 HIGH_COST=1
uint8 SPEED_LIMIT=2
uint8 zone_type
string frame_id
geometry_msgs/Point[] polygon
uint8 cost
float32 speed_limit
bool enabled
