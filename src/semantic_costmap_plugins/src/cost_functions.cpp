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
  if (strategy_name == "add" || strategy_name == "addition") {
    return MergeStrategy::Add;
  }
  if (strategy_name == "overwrite") {
    return MergeStrategy::Overwrite;
  }
  return MergeStrategy::Max;
}

bool modeMatches(const std::string & zone_mode, const std::string & active_mode)
{
  if (zone_mode.empty() || zone_mode == "all" || zone_mode == "*") {
    return true;
  }
  if (active_mode.empty()) {
    return false;
  }
  return zone_mode == active_mode;
}

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
    return std::max(master_cost, layer_cost);
  }

  // Addition mode: keep penalties below lethal by saturation, unless a layer
  // explicitly requests lethal cost.
  if (layer_cost >= LETHAL_OBSTACLE) {
    return LETHAL_OBSTACLE;
  }

  const int sum = static_cast<int>(master_cost) + static_cast<int>(layer_cost);
  return static_cast<unsigned char>(std::min(sum, static_cast<int>(LETHAL_OBSTACLE - 1U)));
}

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
