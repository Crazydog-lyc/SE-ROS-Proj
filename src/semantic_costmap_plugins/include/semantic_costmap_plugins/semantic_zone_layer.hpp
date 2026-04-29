#ifndef SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_ZONE_LAYER_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_ZONE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{

class SemanticZoneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SemanticZoneLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void onFootprintChanged() override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override {return false;}

private:
  void loadZonesFromParameters();
  bool zoneContains(const SemanticZone & zone, double wx, double wy) const;
  unsigned char zoneCostForPoint(const SemanticZone & zone, double wx, double wy) const;
  bool shouldApplyOnCell(unsigned char master_cost, const SemanticZone & zone) const;
  void onTaskModeMessage(const std_msgs::msg::String::SharedPtr msg);

  mutable std::mutex mutex_;
  std::vector<SemanticZone> zones_;
  std::string active_mode_;
  std::string task_mode_topic_;
  MergeStrategy merge_strategy_ {MergeStrategy::Max};
  bool full_map_update_ {true};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_mode_sub_;
};

}  // namespace semantic_costmap_plugins

#endif  // SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_ZONE_LAYER_HPP_
