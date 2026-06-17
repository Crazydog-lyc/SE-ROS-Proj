# ========================================================================
# 文件: src/course_interfaces/msg/BatchTestResult.msg
# 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
# ========================================================================
#
# 【AI-PROMPT】
# BatchTestResult.msg 用于批量测试汇总：case_id、success、elapsed_sec、message 等字段，请生成 rosidl
# 骨架并加简短注释。
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
string case_id
bool success
float32 elapsed_sec
uint32 completed_waypoints
uint32 safety_stop_count
string fail_reason
