#include "nav2_scenario_runner/plugins/congestion_generator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_scenario_runner::plugins
{

std::string CongestionGenerator::name() const
{
  return "congestion";
}

void CongestionGenerator::configure(const rclcpp::Node::SharedPtr & node)
{
  node->declare_parameter("default_trigger_time", default_trigger_time_);
  node->declare_parameter("default_duration", default_duration_);
  node->declare_parameter("default_strength", default_strength_);
  node->get_parameter("default_trigger_time", default_trigger_time_);
  node->get_parameter("default_duration", default_duration_);
  node->get_parameter("default_strength", default_strength_);
}

ScenarioSpec CongestionGenerator::generate(const ScenarioRequest & request)
{
  ScenarioSpec spec;
  spec.case_id = request.case_id;
  spec.scenario_type = request.scenario_type;
  spec.seed = request.seed;
  spec.start_pose = Pose2D{-3.0, 0.0, 0.0};

  spec.walls.push_back({"north_wall", 0.0, 3.0, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"south_wall", 0.0, -3.0, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"east_wall", 3.9, 0.0, 0.2, 6.0, 0.0, 254, true});
  spec.walls.push_back({"west_wall", -3.9, 0.0, 0.2, 6.0, 0.0, 254, true});

  spec.static_obstacles.push_back({"center_island", 0.0, 0.0, 1.2, 1.2, 0.0, 0, true});
  spec.waypoints = {{-2.8, 0.0, 0.0}, {2.8, 0.0, 0.0}};

  if (request.enable_congestion) {
    DynamicEvent event;
    event.type = "congestion";
    event.trigger_time = default_trigger_time_;
    event.duration = default_duration_;
    event.region = RectRegion{"hotspot", 1.2, 0.0, 1.0, 1.6, 0.0, 0, true};
    event.strength = default_strength_;
    spec.events.push_back(event);
  }

  spec.metadata["generator"] = "CongestionGenerator";
  spec.metadata["description"] = "Simple point-to-point scenario with online congestion event.";
  return spec;
}

}  // namespace nav2_scenario_runner::plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_scenario_runner::plugins::CongestionGenerator,
  nav2_scenario_runner::ScenarioGenerator)
