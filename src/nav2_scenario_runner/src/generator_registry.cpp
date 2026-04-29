#include "nav2_scenario_runner/generator_registry.hpp"

namespace nav2_scenario_runner
{

GeneratorRegistry::GeneratorRegistry()
: loader_("nav2_scenario_runner", "nav2_scenario_runner::ScenarioGenerator")
{
}

ScenarioGenerator::Ptr GeneratorRegistry::create(const std::string & class_name)
{
  return loader_.createSharedInstance(class_name);
}

std::vector<std::string> GeneratorRegistry::declaredPlugins()
{
  return loader_.getDeclaredClasses();
}

}  // namespace nav2_scenario_runner
