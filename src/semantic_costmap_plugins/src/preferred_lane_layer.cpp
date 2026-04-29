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

void PreferredLaneLayer::onInitialize()
{
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

void PreferredLaneLayer::loadLanesFromParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PreferredLaneLayer failed to lock lifecycle node while loading parameters");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  lanes_.clear();

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

bool PreferredLaneLayer::shouldApplyOnCell(unsigned char master_cost, const PreferredLane & lane) const
{
  if (!lane.apply_to_unknown && master_cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

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

void PreferredLaneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

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
          // A cell is good if it is close to any preferred lane, so use the minimum
          // penalty among all active lanes.
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

void PreferredLaneLayer::onFootprintChanged()
{
  current_ = false;
}

void PreferredLaneLayer::reset()
{
  current_ = true;
}

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
