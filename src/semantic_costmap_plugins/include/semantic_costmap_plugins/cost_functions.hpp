// ========================================================================
// 文件: src/semantic_costmap_plugins/include/semantic_costmap_plugins/cost_functions.hpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// 基于 Nav2 Humble costmap_2d::Layer，帮我新建 semantic_costmap_plugins 包骨架：SemanticZoneLayer /
// PreferredLaneLayer / DynamicCongestionLayer 三个插件类，继承 CostmapLayer，先实现
// onInitialize、updateBounds、updateCosts 空壳和 pluginlib 导出，附带 geometry_utils、cost_functions
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#pragma once

#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{
namespace cost_functions
{

MergeStrategy parseMergeStrategy(const std::string & strategy_name);  // 解析 max/add/overwrite


bool modeMatches(const std::string & zone_mode, const std::string & active_mode);  // task_mode 过滤

unsigned char mergeCosts(  // 融合 master 与本层代价
  unsigned char master_cost,
  unsigned char layer_cost,
  MergeStrategy strategy);

unsigned char lanePenalty(  // 偏好车道距离惩罚
  double distance_to_lane,
  double corridor_width,
  double inner_penalty,
  double outside_gain,
  double max_penalty);

unsigned char congestionPenalty(  // 动态拥堵圆惩罚
  double distance_to_center,
  double radius,
  double peak_cost,
  double exponent);

}  // namespace cost_functions
}  // namespace semantic_costmap_plugins
