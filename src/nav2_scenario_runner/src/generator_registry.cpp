// ========================================================================
// 文件: src/nav2_scenario_runner/src/generator_registry.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// 帮我 scaffold 一个 nav2_scenario_runner C++ 包：pluginlib 注册 ScenarioGenerator 插件，包含
// generator_registry、scenario_types、scenario_serializer，再写 generate_scenario_node 和 Python
// 侧 generate_cases/run_batch 脚本入口。要求固定 random seed、每个 case 输出独立目录；具体 corridor/room
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
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
