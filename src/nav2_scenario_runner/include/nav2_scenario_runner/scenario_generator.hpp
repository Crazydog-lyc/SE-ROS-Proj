// ========================================================================
// 文件: src/nav2_scenario_runner/include/nav2_scenario_runner/scenario_generator.hpp
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
#pragma once

#include <memory>
#include <string>

#include "nav2_scenario_runner/scenario_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_scenario_runner
{

class ScenarioGenerator
{
public:
  using Ptr = std::shared_ptr<ScenarioGenerator>;
  virtual ~ScenarioGenerator() = default;

  virtual std::string name() const = 0;
  virtual void configure(const rclcpp::Node::SharedPtr & node) = 0;
  virtual ScenarioSpec generate(const ScenarioRequest & request) = 0;

};

}  // namespace nav2_scenario_runner
