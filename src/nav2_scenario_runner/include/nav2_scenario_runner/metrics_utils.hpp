// ========================================================================
// 文件: src/nav2_scenario_runner/include/nav2_scenario_runner/metrics_utils.hpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// 帮我 scaffold 一个 nav2_scenario_runner C++ 包：pluginlib 注册 ScenarioGenerator 插件，包含
// generator_registry、scenario_types、scenario_serializer，再写 generate_scenario_node 和 Python
// 侧 generate_cases/run_batch 脚本入口。要求固定 random seed、每个 case 输出独立目录；具体 corridor/room
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#pragma once

#include <cmath>
#include <vector>

#include "nav2_scenario_runner/scenario_types.hpp"

namespace nav2_scenario_runner
{

inline double poseDistance(const Pose2D & a, const Pose2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double waypointPathLength(const std::vector<Pose2D> & waypoints)
{
  if (waypoints.size() < 2) {

    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    total += poseDistance(waypoints[i - 1], waypoints[i]);
  }
  return total;
}

}  // namespace nav2_scenario_runner
