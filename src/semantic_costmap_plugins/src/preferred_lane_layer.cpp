// ========================================================================
// 文件: src/semantic_costmap_plugins/src/preferred_lane_layer.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// PreferredLaneLayer：在车道多边形内降低代价引导路径，参数 lane_polygons + lane_cost。请生成 CostmapLayer 插件骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【模块说明】语义 costmap 插件实现，参数见 config/nav2_params_semantic.yaml
// ========================================================================
#include "semantic_costmap_plugins/preferred_lane_layer.hpp"

#include <algorithm>
#include <stdexcept>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "semantic_costmap_plugins/cost_functions.hpp"
#include "semantic_costmap_plugins/geometry_utils.hpp"
#include "semantic_costmap_plugins/parameter_utils.hpp"

namespace semantic_costmap_plugins
{


PreferredLaneLayer::PreferredLaneLayer()
: active_mode_("all")
{
}

// 插件初始化：读参数、订阅 topic
void PreferredLaneLayer::onInitialize()
{
  // 偏好车道：多边形内降低代价，引导 planner 走主通道
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PreferredLaneLayer failed to lock lifecycle node");
  }

  enabled_ = parameter_utils::getBool(node, name_ + ".enabled", true);
  task_mode_topic_ = parameter_utils::getString(
    node, name_ + ".task_mode_topic", "/semantic_task_mode");
  active_mode_ = parameter_utils::getString(node, name_ + ".default_mode", "all");
  merge_strategy_ = cost_functions::parseMergeStrategy(
    parameter_utils::getString(node, name_ + ".merge_strategy", "add"));
  full_map_update_ = parameter_utils::getBool(node, name_ + ".full_map_update", true);

  loadLanesFromParameters();

  // 任务模式切换时只激活对应 mode 的车道（和 semantic_zone 同一套 topic）
  if (!task_mode_topic_.empty()) {
    task_mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      task_mode_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&PreferredLaneLayer::onTaskModeMessage, this, std::placeholders::_1));
  }

  current_ = true;
  matchSize();

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] initialized with %zu preferred lanes, active mode='%s'",
    name_.c_str(), lanes_.size(), active_mode_.c_str());
}

// 从参数加载偏好车道折线
void PreferredLaneLayer::loadLanesFromParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PreferredLaneLayer failed to lock lifecycle node while loading parameters");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lanes_.clear();

  // lane_names 下列出要加载的车道，每个车道一段 polyline
  const auto lane_names = parameter_utils::getStringArray(node, name_ + ".lane_names", {});
  for (const auto & lane_name : lane_names) {
    const std::string prefix = name_ + ".lanes." + lane_name + ".";
    PreferredLane lane;
    lane.name = lane_name;
    lane.enabled = parameter_utils::getBool(node, prefix + "enabled", true);
    lane.mode = parameter_utils::getString(node, prefix + "mode", "all");
    lane.corridor_width = parameter_utils::getDouble(node, prefix + "corridor_width", 1.0);
    lane.inner_penalty = parameter_utils::getDouble(node, prefix + "inner_penalty", 0.0);
    lane.outside_gain = parameter_utils::getDouble(node, prefix + "outside_gain", 25.0);
    lane.max_penalty = parameter_utils::getDouble(node, prefix + "max_penalty", 120.0);
    lane.apply_to_unknown = parameter_utils::getBool(node, prefix + "apply_to_unknown", false);

    const auto xs = parameter_utils::getDoubleArray(node, prefix + "points_x", {});
    const auto ys = parameter_utils::getDoubleArray(node, prefix + "points_y", {});
    if (xs.size() != ys.size() || xs.size() < 2U) {
      // 至少两个点才能成线，否则跳过
      RCLCPP_WARN(
        node->get_logger(),
        "[%s] lane '%s' ignored because polyline points are invalid",
        name_.c_str(), lane_name.c_str());
      continue;
    }

    lane.points.reserve(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) {
      lane.points.push_back(Point2D{xs[i], ys[i]});
    }

    lanes_.push_back(lane);
  }
}

// shouldApplyOnCell 接口
bool PreferredLaneLayer::shouldApplyOnCell(unsigned char master_cost, const PreferredLane & lane) const
{
  if (!lane.apply_to_unknown && master_cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

// updateBounds 接口
void PreferredLaneLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !full_map_update_) {
    return;
  }

  auto * master = layered_costmap_->getCostmap();
  if (master == nullptr) {
    return;
  }

  const double origin_x = master->getOriginX();
  const double origin_y = master->getOriginY();
  const double max_x_world = origin_x + master->getResolution() * master->getSizeInCellsX();
  const double max_y_world = origin_y + master->getResolution() * master->getSizeInCellsY();

  *min_x = std::min(*min_x, origin_x);
  *min_y = std::min(*min_y, origin_y);
  *max_x = std::max(*max_x, max_x_world);
  *max_y = std::max(*max_y, max_y_world);
}

// updateCosts 接口
void PreferredLaneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  // 拷贝一份车道列表，避免 updateCosts 和参数回调抢锁
  std::vector<PreferredLane> lanes_copy;
  std::string active_mode_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lanes_copy = lanes_;
    active_mode_copy = active_mode_;
  }

  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  // 双层循环遍历 costmap 更新窗口
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      master_grid.mapToWorld(i, j, wx, wy);

      const unsigned char master_cost = master_grid.getCost(i, j);
      unsigned char best_lane_penalty = 0U;
      bool first_match = true;

      for (const auto & lane : lanes_copy) {
        if (!lane.enabled || !cost_functions::modeMatches(lane.mode, active_mode_copy)) {
          continue;
        }
        if (!shouldApplyOnCell(master_cost, lane) || !geometry_utils::validPolyline(lane.points)) {
          continue;
        }

        const double distance_to_lane =
          geometry_utils::distancePointToPolyline(wx, wy, lane.points);
        const unsigned char penalty = cost_functions::lanePenalty(
          distance_to_lane,
          lane.corridor_width,
          lane.inner_penalty,
          lane.outside_gain,
          lane.max_penalty);

        if (first_match) {
          best_lane_penalty = penalty;
          first_match = false;
        } else {
          // 离任意一条偏好车道越近 penalty 越小，取 min
          best_lane_penalty = std::min(best_lane_penalty, penalty);
        }
      }

      if (!first_match && best_lane_penalty > 0U) {
        master_grid.setCost(
          i, j,
          cost_functions::mergeCosts(master_cost, best_lane_penalty, merge_strategy_));
      }
    }
  }
}

// footprint 变化时标记需要重算
void PreferredLaneLayer::onFootprintChanged()
{
  current_ = false;
}

// layer reset 回调
void PreferredLaneLayer::reset()
{
  current_ = true;
}

// 与 master costmap 尺寸对齐
void PreferredLaneLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();
  if (master != nullptr) {
    this->resizeMap(
      master->getSizeInCellsX(),
      master->getSizeInCellsY(),
      master->getResolution(),
      master->getOriginX(),
      master->getOriginY());
  }
}

// 与 semantic_zone 共用 task_mode topic
void PreferredLaneLayer::onTaskModeMessage(const std_msgs::msg::String::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_mode_ = msg->data.empty() ? std::string("all") : msg->data;
  }
  current_ = false;
}

}  // namespace semantic_costmap_plugins

PLUGINLIB_EXPORT_CLASS(semantic_costmap_plugins::PreferredLaneLayer, nav2_costmap_2d::Layer)
