// ========================================================================
// 文件: src/nav2_scenario_runner/test/test_metrics_utils.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// 请为这个模块生成 gtest/launch_test 骨架：一个 smoke 用例断言核心函数不抛异常，具体断言我后面补。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include <gtest/gtest.h>

#include "nav2_scenario_runner/metrics_utils.hpp"

TEST(MetricsUtils, ComputesPathLength)
{
  using nav2_scenario_runner::Pose2D;
  std::vector<Pose2D> waypoints{
    {0.0, 0.0, 0.0},
    {3.0, 4.0, 0.0},
    {6.0, 4.0, 0.0}
  };

  EXPECT_DOUBLE_EQ(nav2_scenario_runner::waypointPathLength(waypoints), 8.0);
}

