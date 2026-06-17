// ========================================================================
// 文件: src/semantic_costmap_plugins/include/semantic_costmap_plugins/parameter_utils.hpp
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace semantic_costmap_plugins
{
namespace parameter_utils
{

// 读 lifecycle node 参数的薄封装，避免 layer 里重复 declare
template<typename T>
T declareAndGet(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  T value = default_value;
  node->get_parameter(name, value);
  return value;
}

inline std::string getString(  // 字符串参数
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const std::string & default_value)
{
  return declareAndGet<std::string>(node, name, default_value);
}

inline bool getBool(  // 布尔参数
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,

  bool default_value)
{
  return declareAndGet<bool>(node, name, default_value);
}

inline double getDouble(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  double default_value)
{
  return declareAndGet<double>(node, name, default_value);
}

inline int getInt(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  int default_value)
{
  return declareAndGet<int>(node, name, default_value);
}

inline std::vector<double> getDoubleArray(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const std::vector<double> & default_value = {})
{
  return declareAndGet<std::vector<double>>(node, name, default_value);
}

inline std::vector<std::string> getStringArray(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const std::vector<std::string> & default_value = {})
{
  return declareAndGet<std::vector<std::string>>(node, name, default_value);
}

inline bool parameterExists(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name)
{
  return node->has_parameter(name);
}

}  // namespace parameter_utils
}  // namespace semantic_costmap_plugins
