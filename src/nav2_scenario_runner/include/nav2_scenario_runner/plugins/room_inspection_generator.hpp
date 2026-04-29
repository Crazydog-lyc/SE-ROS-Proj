#pragma once

#include "nav2_scenario_runner/scenario_generator.hpp"

namespace nav2_scenario_runner::plugins
{

class RoomInspectionGenerator : public ScenarioGenerator
{
public:
  std::string name() const override;
  void configure(const rclcpp::Node::SharedPtr & node) override;
  ScenarioSpec generate(const ScenarioRequest & request) override;

private:
  double room_size_{2.2};
  double doorway_width_{0.9};
};

}  // namespace nav2_scenario_runner::plugins
