// ========================================================================
// 文件: src/semantic_costmap_plugins/src/dynamic_congestion_layer.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// DynamicCongestionLayer：订阅 congestion 话题，在圆形区域内动态提高代价，带 decay。请生成订阅回调和 updateCosts 框架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【模块说明】语义 costmap 插件实现，参数见 config/nav2_params_semantic.yaml
// ========================================================================
#include "semantic_costmap_plugins/dynamic_congestion_layer.hpp"

#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "semantic_costmap_plugins/cost_functions.hpp"
#include "semantic_costmap_plugins/geometry_utils.hpp"
#include "semantic_costmap_plugins/parameter_utils.hpp"

namespace semantic_costmap_plugins
{


DynamicCongestionLayer::DynamicCongestionLayer() = default;

// 插件初始化：读参数、订阅 topic
void DynamicCongestionLayer::onInitialize()
{
  // 订阅 /semantic_congestion_events，动态抬高局部代价
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("DynamicCongestionLayer failed to lock lifecycle node");
  }

  enabled_ = parameter_utils::getBool(node, name_ + ".enabled", true);
  event_topic_ = parameter_utils::getString(
    node, name_ + ".event_topic", "/semantic_congestion_events");
  merge_strategy_ = cost_functions::parseMergeStrategy(
    parameter_utils::getString(node, name_ + ".merge_strategy", "max"));
  full_map_update_ = parameter_utils::getBool(node, name_ + ".full_map_update", true);
  apply_to_unknown_ = parameter_utils::getBool(node, name_ + ".apply_to_unknown", false);
  default_exponent_ = parameter_utils::getDouble(node, name_ + ".default_exponent", 1.5);
  default_ttl_sec_ = parameter_utils::getDouble(node, name_ + ".default_ttl_sec", 8.0);

  events_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    event_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DynamicCongestionLayer::onEventMessage, this, std::placeholders::_1));

  current_ = true;
  matchSize();

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] listening for congestion events on '%s'",
    name_.c_str(), event_topic_.c_str());
}

// 收到拥堵事件消息，追加到 events_
void DynamicCongestionLayer::onEventMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  const auto & data = msg->data;
  if (data.empty()) {
    return;
  }

  if ((data.size() % 6U) != 0U && (data.size() % 5U) != 0U) {
    // 每条事件 5 或 6 个 float：x,y,r,peak,ttl[,exponent]
    RCLCPP_WARN(
      node->get_logger(),
      "[%s] ignored congestion message because data length %zu is not a multiple of 5 or 6",
      name_.c_str(), data.size());
    return;
  }

  const bool six_fields = ((data.size() % 6U) == 0U);
  const std::size_t stride = six_fields ? 6U : 5U;
  const double now_sec = node->now().seconds();

  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < data.size(); index += stride) {
    DynamicCongestionEvent event;
    event.x = data[index + 0U];
    event.y = data[index + 1U];
    event.radius = std::max(0.01, static_cast<double>(data[index + 2U]));
    event.peak_cost = std::clamp(static_cast<double>(data[index + 3U]), 0.0, 252.0);
    event.ttl_sec = std::max(
      0.1,
      static_cast<double>(data[index + 4U]));
    event.exponent = six_fields ?
      std::max(0.1, static_cast<double>(data[index + 5U])) :
      default_exponent_;
    event.created_at_sec = now_sec;
    events_.push_back(event);
  }

  current_ = false;
}

// 清理 TTL 过期的动态事件
void DynamicCongestionLayer::pruneExpiredEvents(double now_sec)
{
  // TTL 过了就删，避免 events_ 无限涨
  events_.erase(
    std::remove_if(
      events_.begin(), events_.end(),
      [now_sec](const DynamicCongestionEvent & event) {
        return (now_sec - event.created_at_sec) > event.ttl_sec;
      }),
    events_.end());
}

// shouldApplyOnCell 接口
bool DynamicCongestionLayer::shouldApplyOnCell(unsigned char master_cost) const
{
  if (!apply_to_unknown_ && master_cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

// updateBounds 接口
void DynamicCongestionLayer::updateBounds(
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
void DynamicCongestionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::vector<DynamicCongestionEvent> events_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pruneExpiredEvents(node->now().seconds());
    events_copy = events_;
  }

  if (events_copy.empty()) {
    return;
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
      if (!shouldApplyOnCell(master_cost)) {
        continue;
      }

      unsigned char best_penalty = 0U;
      for (const auto & event : events_copy) {
        // 多个拥堵圆重叠时取最大 penalty
        const double d = geometry_utils::distance(wx, wy, event.x, event.y);
        best_penalty = std::max(
          best_penalty,
          cost_functions::congestionPenalty(d, event.radius, event.peak_cost, event.exponent));
      }

      if (best_penalty > 0U) {
        master_grid.setCost(
          i, j,
          cost_functions::mergeCosts(master_cost, best_penalty, merge_strategy_));
      }
    }
  }
}

// footprint 变化时标记需要重算
void DynamicCongestionLayer::onFootprintChanged()
{
  current_ = false;
}

// layer reset 回调
void DynamicCongestionLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.clear();
  current_ = true;
}

// 与 master costmap 尺寸对齐
void DynamicCongestionLayer::matchSize()
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

}  // namespace semantic_costmap_plugins

PLUGINLIB_EXPORT_CLASS(semantic_costmap_plugins::DynamicCongestionLayer, nav2_costmap_2d::Layer)
