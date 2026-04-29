#ifndef SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_

#include <vector>

#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{
namespace geometry_utils
{

double deg2rad(double degrees);
double distance(double x1, double y1, double x2, double y2);
double distanceSquared(double x1, double y1, double x2, double y2);

Point2D rotatePointAroundOrigin(const Point2D & point, double yaw_rad);
Point2D translatePoint(const Point2D & point, double tx, double ty);

bool pointInCircle(
  double px, double py,
  double cx, double cy,
  double radius);

bool pointInRotatedRectangle(
  double px, double py,
  double cx, double cy,
  double width, double height,
  double yaw_rad);

bool pointInPolygon(
  double px, double py,
  const std::vector<Point2D> & polygon);

double distancePointToSegment(
  double px, double py,
  const Point2D & a,
  const Point2D & b);

double distancePointToPolyline(
  double px, double py,
  const std::vector<Point2D> & points);

bool validPolyline(const std::vector<Point2D> & points);

}  // namespace geometry_utils
}  // namespace semantic_costmap_plugins

#endif  // SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_
