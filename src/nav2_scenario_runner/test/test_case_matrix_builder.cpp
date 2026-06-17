// ========================================================================
// 文件: src/nav2_scenario_runner/test/test_case_matrix_builder.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// 请为这个模块生成 gtest/launch_test 骨架：一个 smoke 用例断言核心函数不抛异常，具体断言我后面补。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include <gtest/gtest.h>

#include "nav2_scenario_runner/scenario_serializer.hpp"
#include "nav2_scenario_runner/scenario_types.hpp"

TEST(ScenarioSerializer, ProducesWaypointsYaml)
{
  nav2_scenario_runner::ScenarioSpec spec;
  spec.case_id = "demo_case";
  spec.waypoints.push_back({0.0, 0.0, 0.0});
  spec.waypoints.push_back({1.0, 0.0, 0.0});


  auto node = nav2_scenario_runner::ScenarioSerializer::toWaypointsYaml(spec);
  ASSERT_TRUE(node["waypoints"]);
  EXPECT_EQ(node["waypoints"].size(), 2u);
}

TEST(ScenarioSerializer, SanitizesCaseId)
{
  const auto s = nav2_scenario_runner::ScenarioSerializer::sanitizeFileStem("case 01/room");
  EXPECT_EQ(s, "case_01_room");
}
