#pragma once

#include "nav2_scenario_runner/scenario_generator.hpp"

namespace nav2_scenario_runner::plugins
{

class CorridorGenerator : public ScenarioGenerator
{
public:
  std::string name() const override;
  void configure(const rclcpp::Node::SharedPtr & node) override;
  ScenarioSpec generate(const ScenarioRequest & request) override;

private:
  double corridor_width_{1.8};
  double wall_thickness_{0.2};
  double keepout_cost_{200.0};
};

}  // namespace nav2_scenario_runner::plugins
