#pragma once

#include <memory>
#include <string>

#include "nav2_scenario_runner/scenario_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_scenario_runner
{

class ScenarioGenerator
{
public:
  using Ptr = std::shared_ptr<ScenarioGenerator>;
  virtual ~ScenarioGenerator() = default;

  virtual std::string name() const = 0;
  virtual void configure(const rclcpp::Node::SharedPtr & node) = 0;
  virtual ScenarioSpec generate(const ScenarioRequest & request) = 0;
};

}  // namespace nav2_scenario_runner
