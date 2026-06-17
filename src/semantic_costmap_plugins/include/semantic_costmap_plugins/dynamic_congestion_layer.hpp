// ========================================================================
// 文件: src/semantic_costmap_plugins/include/semantic_costmap_plugins/dynamic_congestion_layer.hpp
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

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{

class DynamicCongestionLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DynamicCongestionLayer();


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
  void onEventMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void pruneExpiredEvents(double now_sec);
  bool shouldApplyOnCell(unsigned char master_cost) const;

  mutable std::mutex mutex_;
  std::vector<DynamicCongestionEvent> events_;
  std::string event_topic_;
  MergeStrategy merge_strategy_ {MergeStrategy::Max};
  bool full_map_update_ {true};
  bool apply_to_unknown_ {false};
  double default_exponent_ {1.0};
  double default_ttl_sec_ {8.0};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr events_sub_;
};

}  // namespace semantic_costmap_plugins
