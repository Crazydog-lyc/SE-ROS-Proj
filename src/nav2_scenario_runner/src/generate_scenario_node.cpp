#include <memory>
#include <string>

#include "nav2_scenario_runner/generator_registry.hpp"
#include "nav2_scenario_runner/scenario_serializer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("generate_scenario_node");

  node->declare_parameter<std::string>("generator_plugin", "nav2_scenario_runner::plugins::CorridorGenerator");
  node->declare_parameter<std::string>("case_id", "case_000");
  node->declare_parameter<std::string>("scenario_type", "corridor");
  node->declare_parameter<int>("seed", 42);
  node->declare_parameter<int>("waypoint_count", 4);
  node->declare_parameter<int>("room_count", 3);
  node->declare_parameter<int>("obstacle_density", 2);
  node->declare_parameter<bool>("enable_semantic_regions", true);
  node->declare_parameter<bool>("enable_faults", false);
  node->declare_parameter<bool>("enable_congestion", false);
  node->declare_parameter<double>("map_width", 8.0);
  node->declare_parameter<double>("map_height", 8.0);
  node->declare_parameter<std::string>("output_dir", "./generated_cases");

  nav2_scenario_runner::ScenarioRequest req;
  std::string generator_plugin;
  std::string output_dir;

  node->get_parameter("generator_plugin", generator_plugin);
  node->get_parameter("case_id", req.case_id);
  node->get_parameter("scenario_type", req.scenario_type);
  node->get_parameter("seed", req.seed);
  node->get_parameter("waypoint_count", req.waypoint_count);
  node->get_parameter("room_count", req.room_count);
  node->get_parameter("obstacle_density", req.obstacle_density);
  node->get_parameter("enable_semantic_regions", req.enable_semantic_regions);
  node->get_parameter("enable_faults", req.enable_faults);
  node->get_parameter("enable_congestion", req.enable_congestion);
  node->get_parameter("map_width", req.map_width);
  node->get_parameter("map_height", req.map_height);
  node->get_parameter("output_dir", output_dir);

  try {
    nav2_scenario_runner::GeneratorRegistry registry;
    auto generator = registry.create(generator_plugin);
    generator->configure(node);
    auto spec = generator->generate(req);
    nav2_scenario_runner::ScenarioSerializer::writeScenarioFiles(spec, output_dir);
    RCLCPP_INFO(node->get_logger(), "Generated scenario %s into %s", req.case_id.c_str(), output_dir.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Scenario generation failed: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
