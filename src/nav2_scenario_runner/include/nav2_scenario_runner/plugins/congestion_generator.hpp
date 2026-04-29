#pragma once

#include "nav2_scenario_runner/scenario_generator.hpp"

namespace nav2_scenario_runner::plugins
{

class CongestionGenerator : public ScenarioGenerator
{
public:
  std::string name() const override;
  void configure(const rclcpp::Node::SharedPtr & node) override;
  ScenarioSpec generate(const ScenarioRequest & request) override;

private:
  double default_trigger_time_{10.0};
  double default_duration_{8.0};
  double default_strength_{140.0};
};

}  // namespace nav2_scenario_runner::plugins
