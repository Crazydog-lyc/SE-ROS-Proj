// ========================================================================
// 文件: src/semantic_costmap_plugins/src/cost_functions.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// cost_functions：merge_strategy max/add/replace，把 zone cost 写入 master_grid。请生成函数骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【模块说明】语义 costmap 插件实现，参数见 config/nav2_params_semantic.yaml
// ========================================================================
#include "semantic_costmap_plugins/cost_functions.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace semantic_costmap_plugins
{
namespace cost_functions
{

MergeStrategy parseMergeStrategy(const std::string & strategy_name)
{
  // yaml 里写 add/max/overwrite，统一解析成枚举
  if (strategy_name == "add" || strategy_name == "addition") {
    return MergeStrategy::Add;
  }
  if (strategy_name == "overwrite") {
    return MergeStrategy::Overwrite;
  }
  return MergeStrategy::Max;
}


// modeMatches 接口
// TODO[李熠城]：FR-C-03 任务模式过滤，决定 zone/lane 是否参与代价计算
bool modeMatches(const std::string & zone_mode, const std::string & active_mode)
{
  // zone.mode 为 all 时任何任务模式都生效
  if (zone_mode.empty() || zone_mode == "all" || zone_mode == "*") {
    return true;
  }
  if (active_mode.empty()) {
    return false;
  }
  return zone_mode == active_mode;
}

// TODO[李熠城]：FR-C-03 max/add/overwrite 三种代价融合策略
unsigned char mergeCosts(
  unsigned char master_cost,
  unsigned char layer_cost,
  MergeStrategy strategy)
{
  using nav2_costmap_2d::LETHAL_OBSTACLE;
  using nav2_costmap_2d::NO_INFORMATION;

  if (layer_cost == 0U || layer_cost == NO_INFORMATION) {
    return master_cost;
  }

  if (strategy == MergeStrategy::Overwrite) {
    return layer_cost;
  }

  if (master_cost == NO_INFORMATION) {
    return layer_cost;
  }

  if (strategy == MergeStrategy::Max) {
    // 默认策略：取 master 与本层代价的较大值
    return std::max(master_cost, layer_cost);
  }

  // add 模式：累加但饱和到 lethal 以下，除非本层明确要求 lethal
  if (layer_cost >= LETHAL_OBSTACLE) {
    return LETHAL_OBSTACLE;
  }

  const int sum = static_cast<int>(master_cost) + static_cast<int>(layer_cost);
  return static_cast<unsigned char>(std::min(sum, static_cast<int>(LETHAL_OBSTACLE - 1U)));
}

// TODO[李熠城]：FR-C-04 偏好车道走廊内外分段惩罚曲线
unsigned char lanePenalty(
  double distance_to_lane,
  double corridor_width,
  double inner_penalty,
  double outside_gain,
  double max_penalty)
{
  if (!std::isfinite(distance_to_lane) || corridor_width <= 0.0) {
    return 0U;
  }

  const double half_width = corridor_width * 0.5;
  double penalty = inner_penalty;

  if (distance_to_lane > half_width) {
    penalty += (distance_to_lane - half_width) * outside_gain;
  }

  penalty = std::clamp(penalty, 0.0, max_penalty);
  return static_cast<unsigned char>(std::lround(penalty));
}

// TODO[李熠城]：FR-C-05 动态拥堵圆形衰减代价，exponent 控制边缘陡峭度
unsigned char congestionPenalty(
  double distance_to_center,
  double radius,
  double peak_cost,
  double exponent)
{
  if (radius <= 0.0 || distance_to_center > radius || peak_cost <= 0.0) {
    return 0U;
  }

  const double normalized = 1.0 - (distance_to_center / radius);
  const double shaped = std::pow(std::max(0.0, normalized), std::max(0.1, exponent));
  const double penalty = std::clamp(shaped * peak_cost, 0.0, 252.0);
  return static_cast<unsigned char>(std::lround(penalty));
}

}  // namespace cost_functions
}  // namespace semantic_costmap_plugins
