# ========================================================================
# 文件: src/course_interfaces/msg/MissionState.msg
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 帮我写 MissionState.msg：包含 stamp、mission_id、current_waypoint_index、total_waypoints、state
# 字符串、progress_percent、detail，供 /mission/state 话题发布任务进度。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
string mission_id
uint32 current_waypoint_index
uint32 total_waypoints
string state
float32 progress_percent
string detail
