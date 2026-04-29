#pragma once

#include "nav2_scenario_runner/scenario_generator.hpp"

namespace nav2_scenario_runner::plugins
{

class FaultInjectionGenerator : public ScenarioGenerator
{
public:
  std::string name() const override;
  void configure(const rclcpp::Node::SharedPtr & node) override;
  ScenarioSpec generate(const ScenarioRequest & request) override;

private:
  double fault_trigger_time_{12.0};
  double fault_duration_{6.0};
};

}  // namespace nav2_scenario_runner::plugins
