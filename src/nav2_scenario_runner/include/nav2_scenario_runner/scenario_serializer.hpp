#pragma once

#include <string>

#include "nav2_scenario_runner/scenario_types.hpp"
#include "yaml-cpp/yaml.h"

namespace nav2_scenario_runner
{

class ScenarioSerializer
{
public:
  static YAML::Node toYaml(const ScenarioSpec & spec);
  static YAML::Node toWaypointsYaml(const ScenarioSpec & spec);
  static YAML::Node toSemanticRegionsYaml(const ScenarioSpec & spec);
  static YAML::Node toFaultEventsYaml(const ScenarioSpec & spec);
  static void writeScenarioFiles(const ScenarioSpec & spec, const std::string & output_dir);
  static std::string sanitizeFileStem(const std::string & stem);
};

}  // namespace nav2_scenario_runner
