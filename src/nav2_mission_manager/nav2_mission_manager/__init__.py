# ========================================================================
# 文件: src/nav2_mission_manager/nav2_mission_manager/__init__.py
# 负责人: 徐梓鸣 | 需求: FR-B | PPT: 第15-16页 任务管理
# ========================================================================
#
# 【AI-PROMPT】
# 我在 ROS2 Humble 上做 Nav2 课程项目，需要新建 nav2_mission_manager 包。请帮我搭 Python Action Server
# 骨架：/mission/run 用 course_interfaces/RunMission，ReentrantCallbackGroup +
# MultiThreadedExecutor，发布 /mission/state，订阅 /safety/state；先把
#
# 【AI-SCOPE】import · declare · register · 插件/接口空壳
# ========================================================================
# =============================================================================
# src/nav2_mission_manager/nav2_mission_manager/__init__.py
# 负责人: 徐梓鸣  |  需求域: FR-B
# =============================================================================
# 【AI】提示词: "我在 ROS2 Humble 上做 Nav2 课程项目，需要新建 nav2_mission_manager 包。请帮我搭 Python Action Server 骨架：/mission/run 用 course_interfaces/RunMission，ReentrantCallbackGroup + MultiThreadedExecutor，发布 /mission/state，订阅 /safety/state；先把 declare_parameter、goal/cancel callback、_execute_callback 主循环框架写好，状态机细节我后面自己实现。"
# -----------------------------------------------------------------------------
# 【AI】依赖导入 / 接口声明 / 框架常量
"""Mission manager package."""

