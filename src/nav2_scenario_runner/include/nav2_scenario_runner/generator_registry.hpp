#pragma once

#include <string>
#include <vector>

#include "nav2_scenario_runner/scenario_generator.hpp"
#include "pluginlib/class_loader.hpp"

namespace nav2_scenario_runner
{

class GeneratorRegistry
{
public:
  GeneratorRegistry();

  ScenarioGenerator::Ptr create(const std::string & class_name);
  std::vector<std::string> declaredPlugins();

private:
  pluginlib::ClassLoader<ScenarioGenerator> loader_;
};

}  // namespace nav2_scenario_runner
