#pragma once

#include <cmath>
#include <vector>

#include "nav2_scenario_runner/scenario_types.hpp"

namespace nav2_scenario_runner
{

inline double poseDistance(const Pose2D & a, const Pose2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

inline double waypointPathLength(const std::vector<Pose2D> & waypoints)
{
  if (waypoints.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    total += poseDistance(waypoints[i - 1], waypoints[i]);
  }
  return total;
}

}  // namespace nav2_scenario_runner
