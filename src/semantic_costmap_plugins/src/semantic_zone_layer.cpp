// ========================================================================
// 文件: src/semantic_costmap_plugins/src/semantic_zone_layer.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// SemanticZoneLayer：从 ROS 参数加载 zones（矩形/圆/多边形），updateCosts 按 active_mode 融合
// keepout/high-cost，订阅 /semantic_task_mode。请生成
// onInitialize、loadZonesFromParameters、updateCosts 框架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【模块说明】语义 costmap 插件实现，参数见 config/nav2_params_semantic.yaml
// ========================================================================
#include "semantic_costmap_plugins/semantic_zone_layer.hpp"

#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "semantic_costmap_plugins/cost_functions.hpp"
#include "semantic_costmap_plugins/geometry_utils.hpp"
#include "semantic_costmap_plugins/parameter_utils.hpp"

namespace semantic_costmap_plugins
{


SemanticZoneLayer::SemanticZoneLayer()
: active_mode_("all")
{
}

// 插件初始化：读参数、订阅 topic
void SemanticZoneLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SemanticZoneLayer failed to lock lifecycle node");
  }

  // 参数都在 nav2_params_semantic.yaml 的 plugin 段里
  enabled_ = parameter_utils::getBool(node, name_ + ".enabled", true);
  task_mode_topic_ = parameter_utils::getString(
    node, name_ + ".task_mode_topic", "/semantic_task_mode");
  active_mode_ = parameter_utils::getString(node, name_ + ".default_mode", "all");
  merge_strategy_ = cost_functions::parseMergeStrategy(
    parameter_utils::getString(node, name_ + ".merge_strategy", "max"));
  full_map_update_ = parameter_utils::getBool(node, name_ + ".full_map_update", true);

  loadZonesFromParameters();

  if (!task_mode_topic_.empty()) {
    task_mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      task_mode_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SemanticZoneLayer::onTaskModeMessage, this, std::placeholders::_1));
  }

  current_ = true;
  matchSize();

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] initialized with %zu semantic zones, active mode='%s'",
    name_.c_str(), zones_.size(), active_mode_.c_str());
}

// 从参数服务器加载 zone 列表
void SemanticZoneLayer::loadZonesFromParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SemanticZoneLayer failed to lock lifecycle node while loading parameters");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  zones_.clear();

  // zone_names 列表决定加载哪些子段 zones.<name>.*
  const auto zone_names = parameter_utils::getStringArray(node, name_ + ".zone_names", {});

  for (const auto & zone_name : zone_names) {
    const std::string prefix = name_ + ".zones." + zone_name + ".";
    // 每个 zone 可以是 rect/circle/polygon
    SemanticZone zone;
    zone.name = zone_name;
    zone.enabled = parameter_utils::getBool(node, prefix + "enabled", true);
    zone.mode = parameter_utils::getString(node, prefix + "mode", "all");
    zone.cost = static_cast<unsigned char>(
      std::clamp(parameter_utils::getInt(node, prefix + "cost", 0), 0, 254));
    zone.apply_to_unknown = parameter_utils::getBool(node, prefix + "apply_to_unknown", false);

    const std::string shape = parameter_utils::getString(node, prefix + "shape", "rectangle");
    if (shape == "circle") {
      zone.shape = ShapeType::Circle;
      zone.x = parameter_utils::getDouble(node, prefix + "x", 0.0);
      zone.y = parameter_utils::getDouble(node, prefix + "y", 0.0);
      zone.radius = parameter_utils::getDouble(node, prefix + "radius", 0.0);
    } else if (shape == "polygon") {
      zone.shape = ShapeType::Polygon;
      const auto xs = parameter_utils::getDoubleArray(node, prefix + "points_x", {});
      const auto ys = parameter_utils::getDoubleArray(node, prefix + "points_y", {});
      if (xs.size() != ys.size() || xs.size() < 3U) {
        RCLCPP_WARN(
          node->get_logger(),
          "[%s] zone '%s' ignored because polygon points are invalid",
          name_.c_str(), zone_name.c_str());
        continue;
      }

      zone.polygon.reserve(xs.size());
      for (std::size_t i = 0; i < xs.size(); ++i) {
        zone.polygon.push_back(Point2D{xs[i], ys[i]});
      }
    } else {
      zone.shape = ShapeType::Rectangle;
      zone.x = parameter_utils::getDouble(node, prefix + "x", 0.0);
      zone.y = parameter_utils::getDouble(node, prefix + "y", 0.0);
      zone.width = parameter_utils::getDouble(node, prefix + "width", 0.0);
      zone.height = parameter_utils::getDouble(node, prefix + "height", 0.0);
      zone.yaw = parameter_utils::getDouble(node, prefix + "yaw", 0.0);
    }

    zones_.push_back(zone);  // 加入本 layer 的 zone 列表
  }
}

// TODO[李熠城]：FR-C-02 按圆/多边形/旋转矩形分发 zone 点包含判定
bool SemanticZoneLayer::zoneContains(const SemanticZone & zone, double wx, double wy) const
{
  switch (zone.shape) {
    case ShapeType::Circle:
      return geometry_utils::pointInCircle(wx, wy, zone.x, zone.y, zone.radius);
    case ShapeType::Polygon:
      return geometry_utils::pointInPolygon(wx, wy, zone.polygon);
    case ShapeType::Rectangle:
    default:
      return geometry_utils::pointInRotatedRectangle(wx, wy, zone.x, zone.y, zone.width, zone.height, zone.yaw);
  }
}

// 根据 zone 形状算单点代价
 unsigned char SemanticZoneLayer::zoneCostForPoint(const SemanticZone & zone, double wx, double wy) const
{
  return zoneContains(zone, wx, wy) ? zone.cost : 0U;
}

// unknown 栅格是否叠加语义代价
 bool SemanticZoneLayer::shouldApplyOnCell(unsigned char master_cost, const SemanticZone & zone) const
{
  if (!zone.apply_to_unknown && master_cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return true;
}

// updateBounds 接口
void SemanticZoneLayer::updateBounds(
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
// TODO[李熠城]：FR-C-02 遍历栅格，按 active_mode 将 zone keepout/软代价融合进 master costmap
void SemanticZoneLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    current_ = true;
    return;
  }

  std::vector<SemanticZone> zones_copy;
  std::string active_mode_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    zones_copy = zones_;
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

      unsigned char best_layer_cost = 0U;
      const unsigned char master_cost = master_grid.getCost(i, j);

      for (const auto & zone : zones_copy) {
        // 任务模式不匹配或未启用的 zone 跳过
        if (!zone.enabled || !cost_functions::modeMatches(zone.mode, active_mode_copy)) {
          continue;
        }
        if (!shouldApplyOnCell(master_cost, zone)) {
          continue;
        }
        best_layer_cost = std::max(best_layer_cost, zoneCostForPoint(zone, wx, wy));
      }

      if (best_layer_cost > 0U) {
        // keepout / soft cost 按 merge_strategy 写入 master
        master_grid.setCost(
          i, j,
          cost_functions::mergeCosts(master_cost, best_layer_cost, merge_strategy_));
      }
    }
  }
  current_ = true;
}

// footprint 变化时标记需要重算
void SemanticZoneLayer::onFootprintChanged()
{
  current_ = true;
}

// layer reset 回调
void SemanticZoneLayer::reset()
{
  current_ = true;
}

// 与 master costmap 尺寸对齐
void SemanticZoneLayer::matchSize()
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

// 收到 task_mode 后刷新 active_mode_
 void SemanticZoneLayer::onTaskModeMessage(const std_msgs::msg::String::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_mode_ = msg->data.empty() ? std::string("all") : msg->data;
  }
  current_ = false;
}

}  // namespace semantic_costmap_plugins

PLUGINLIB_EXPORT_CLASS(semantic_costmap_plugins::SemanticZoneLayer, nav2_costmap_2d::Layer)
