#include "nav2_scenario_runner/scenario_serializer.hpp"

#include <filesystem>
#include <fstream>
#include <regex>

namespace fs = std::filesystem;

namespace nav2_scenario_runner
{

namespace
{

YAML::Node poseToNode(const Pose2D & pose)
{
  YAML::Node node;
  node["x"] = pose.x;
  node["y"] = pose.y;
  node["yaw"] = pose.yaw;
  return node;
}

YAML::Node rectToNode(const RectRegion & rect)
{
  YAML::Node node;
  node["id"] = rect.id;
  node["x"] = rect.x;
  node["y"] = rect.y;
  node["width"] = rect.width;
  node["height"] = rect.height;
  node["yaw"] = rect.yaw;
  node["cost"] = rect.cost;
  node["enabled"] = rect.enabled;
  return node;
}

YAML::Node laneToNode(const PolylineLane & lane)
{
  YAML::Node node;
  node["id"] = lane.id;
  node["points_x"] = lane.points_x;
  node["points_y"] = lane.points_y;
  node["corridor_width"] = lane.corridor_width;
  node["inner_penalty"] = lane.inner_penalty;
  node["outside_gain"] = lane.outside_gain;
  node["max_penalty"] = lane.max_penalty;
  node["enabled"] = lane.enabled;
  return node;
}

YAML::Node eventToNode(const DynamicEvent & event)
{
  YAML::Node node;
  node["type"] = event.type;
  node["trigger_time"] = event.trigger_time;
  node["duration"] = event.duration;
  node["region"] = rectToNode(event.region);
  node["strength"] = event.strength;
  return node;
}

void writeYamlFile(const fs::path & path, const YAML::Node & node)
{
  YAML::Emitter out;
  out << node;
  std::ofstream ofs(path);
  ofs << out.c_str();
}

}  // namespace

YAML::Node ScenarioSerializer::toYaml(const ScenarioSpec & spec)
{
  YAML::Node root;
  root["case_id"] = spec.case_id;
  root["scenario_type"] = spec.scenario_type;
  root["seed"] = spec.seed;
  root["start_pose"] = poseToNode(spec.start_pose);

  for (const auto & wall : spec.walls) {
    root["walls"].push_back(rectToNode(wall));
  }
  for (const auto & obstacle : spec.static_obstacles) {
    root["static_obstacles"].push_back(rectToNode(obstacle));
  }
  for (const auto & keepout : spec.keepout_regions) {
    root["keepout_regions"].push_back(rectToNode(keepout));
  }
  for (const auto & soft : spec.soft_cost_regions) {
    root["soft_cost_regions"].push_back(rectToNode(soft));
  }
  for (const auto & lane : spec.lanes) {
    root["lanes"].push_back(laneToNode(lane));
  }
  for (const auto & waypoint : spec.waypoints) {
    root["waypoints"].push_back(poseToNode(waypoint));
  }
  for (const auto & event : spec.events) {
    root["events"].push_back(eventToNode(event));
  }
  for (const auto & item : spec.metadata) {
    root["metadata"][item.first] = item.second;
  }
  return root;
}

YAML::Node ScenarioSerializer::toWaypointsYaml(const ScenarioSpec & spec)
{
  YAML::Node root;
  root["case_id"] = spec.case_id;
  for (const auto & waypoint : spec.waypoints) {
    root["waypoints"].push_back(poseToNode(waypoint));
  }
  return root;
}

YAML::Node ScenarioSerializer::toSemanticRegionsYaml(const ScenarioSpec & spec)
{
  YAML::Node root;
  root["case_id"] = spec.case_id;
  for (const auto & keepout : spec.keepout_regions) {
    root["keepout_regions"].push_back(rectToNode(keepout));
  }
  for (const auto & soft : spec.soft_cost_regions) {
    root["soft_cost_regions"].push_back(rectToNode(soft));
  }
  for (const auto & lane : spec.lanes) {
    root["lanes"].push_back(laneToNode(lane));
  }
  return root;
}

YAML::Node ScenarioSerializer::toFaultEventsYaml(const ScenarioSpec & spec)
{
  YAML::Node root;
  root["case_id"] = spec.case_id;
  for (const auto & event : spec.events) {
    root["events"].push_back(eventToNode(event));
  }
  return root;
}

std::string ScenarioSerializer::sanitizeFileStem(const std::string & stem)
{
  static const std::regex invalid(R"([^A-Za-z0-9_\-]+)");
  return std::regex_replace(stem, invalid, "_");
}

void ScenarioSerializer::writeScenarioFiles(const ScenarioSpec & spec, const std::string & output_dir)
{
  fs::create_directories(output_dir);
  const auto base = sanitizeFileStem(spec.case_id);
  writeYamlFile(fs::path(output_dir) / (base + "_scenario.yaml"), toYaml(spec));
  writeYamlFile(fs::path(output_dir) / (base + "_waypoints.yaml"), toWaypointsYaml(spec));
  writeYamlFile(fs::path(output_dir) / (base + "_semantic_regions.yaml"), toSemanticRegionsYaml(spec));
  writeYamlFile(fs::path(output_dir) / (base + "_fault_events.yaml"), toFaultEventsYaml(spec));
}

}  // namespace nav2_scenario_runner
