#ifndef SEMANTIC_COSTMAP_PLUGINS__COST_FUNCTIONS_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__COST_FUNCTIONS_HPP_

#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{
namespace cost_functions
{

MergeStrategy parseMergeStrategy(const std::string & strategy_name);

bool modeMatches(const std::string & zone_mode, const std::string & active_mode);

unsigned char mergeCosts(
  unsigned char master_cost,
  unsigned char layer_cost,
  MergeStrategy strategy);

unsigned char lanePenalty(
  double distance_to_lane,
  double corridor_width,
  double inner_penalty,
  double outside_gain,
  double max_penalty);

unsigned char congestionPenalty(
  double distance_to_center,
  double radius,
  double peak_cost,
  double exponent);

}  // namespace cost_functions
}  // namespace semantic_costmap_plugins

#endif  // SEMANTIC_COSTMAP_PLUGINS__COST_FUNCTIONS_HPP_
