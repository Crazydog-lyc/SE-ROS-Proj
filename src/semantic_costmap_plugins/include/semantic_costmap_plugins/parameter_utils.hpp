#ifndef SEMANTIC_COSTMAP_PLUGINS__PARAMETER_UTILS_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__PARAMETER_UTILS_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace semantic_costmap_plugins
{
namespace parameter_utils
{

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

inline std::string getString(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name,
  const std::string & default_value)
{
  return declareAndGet<std::string>(node, name, default_value);
}

inline bool getBool(
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

#endif  // SEMANTIC_COSTMAP_PLUGINS__PARAMETER_UTILS_HPP_
