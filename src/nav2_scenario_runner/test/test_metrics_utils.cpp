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
