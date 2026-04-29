#ifndef SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_TYPES_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_TYPES_HPP_

#include <string>
#include <vector>

namespace semantic_costmap_plugins
{

struct Point2D
{
  double x {0.0};
  double y {0.0};
};

enum class ShapeType
{
  Rectangle,
  Circle,
  Polygon
};

struct SemanticZone
{
  std::string name;
  std::string mode {"all"};
  ShapeType shape {ShapeType::Rectangle};
  bool enabled {true};

  // Common cost settings
  unsigned char cost {0};
  bool apply_to_unknown {false};

  // Rectangle / circle fields
  double x {0.0};
  double y {0.0};
  double width {0.0};
  double height {0.0};
  double yaw {0.0};
  double radius {0.0};

  // Polygon fields
  std::vector<Point2D> polygon;
};

struct PreferredLane
{
  std::string name;
  std::string mode {"all"};
  bool enabled {true};

  std::vector<Point2D> points;
  double corridor_width {1.0};
  double inner_penalty {0.0};
  double outside_gain {25.0};
  double max_penalty {120.0};
  bool apply_to_unknown {false};
};

struct DynamicCongestionEvent
{
  double x {0.0};
  double y {0.0};
  double radius {0.5};
  double peak_cost {100.0};
  double ttl_sec {8.0};
  double exponent {1.0};
  double created_at_sec {0.0};
};

enum class MergeStrategy
{
  Max,
  Add,
  Overwrite
};

}  // namespace semantic_costmap_plugins

#endif  // SEMANTIC_COSTMAP_PLUGINS__SEMANTIC_TYPES_HPP_
