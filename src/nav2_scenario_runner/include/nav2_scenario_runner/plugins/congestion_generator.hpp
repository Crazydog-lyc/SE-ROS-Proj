// ========================================================================
// 文件: src/nav2_scenario_runner/include/nav2_scenario_runner/plugins/congestion_generator.hpp
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

#include "nav2_scenario_runner/scenario_generator.hpp"

namespace nav2_scenario_runner::plugins
{

class CongestionGenerator : public ScenarioGenerator
{
public:
  std::string name() const override;

  void configure(const rclcpp::Node::SharedPtr & node) override;
  ScenarioSpec generate(const ScenarioRequest & request) override;

private:
  double default_trigger_time_{10.0};
  double default_duration_{8.0};
  double default_strength_{140.0};
};

}  // namespace nav2_scenario_runner::plugins
