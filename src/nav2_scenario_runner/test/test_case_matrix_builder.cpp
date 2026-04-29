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
