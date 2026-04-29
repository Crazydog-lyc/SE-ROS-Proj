#include "semantic_costmap_plugins/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace semantic_costmap_plugins
{
namespace geometry_utils
{

double deg2rad(double degrees)
{
  constexpr double kPi = 3.14159265358979323846;
  return degrees * kPi / 180.0;
}

double distance(double x1, double y1, double x2, double y2)
{
  return std::sqrt(distanceSquared(x1, y1, x2, y2));
}

double distanceSquared(double x1, double y1, double x2, double y2)
{
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

Point2D rotatePointAroundOrigin(const Point2D & point, double yaw_rad)
{
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  return Point2D{
    point.x * c - point.y * s,
    point.x * s + point.y * c};
}

Point2D translatePoint(const Point2D & point, double tx, double ty)
{
  return Point2D{point.x + tx, point.y + ty};
}

bool pointInCircle(
  double px, double py,
  double cx, double cy,
  double radius)
{
  if (radius <= 0.0) {
    return false;
  }
  return distanceSquared(px, py, cx, cy) <= (radius * radius);
}

bool pointInRotatedRectangle(
  double px, double py,
  double cx, double cy,
  double width, double height,
  double yaw_rad)
{
  if (width <= 0.0 || height <= 0.0) {
    return false;
  }

  // Transform the query point into the rectangle frame.
  const double dx = px - cx;
  const double dy = py - cy;
  const double c = std::cos(-yaw_rad);
  const double s = std::sin(-yaw_rad);

  const double local_x = dx * c - dy * s;
  const double local_y = dx * s + dy * c;

  return std::abs(local_x) <= width * 0.5 && std::abs(local_y) <= height * 0.5;
}

bool pointInPolygon(
  double px, double py,
  const std::vector<Point2D> & polygon)
{
  if (polygon.size() < 3U) {
    return false;
  }

  bool inside = false;
  std::size_t j = polygon.size() - 1U;

  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const bool intersects =
      ((polygon[i].y > py) != (polygon[j].y > py)) &&
      (px < (polygon[j].x - polygon[i].x) * (py - polygon[i].y) /
      ((polygon[j].y - polygon[i].y) + std::numeric_limits<double>::epsilon()) + polygon[i].x);

    if (intersects) {
      inside = !inside;
    }
    j = i;
  }

  return inside;
}

double distancePointToSegment(
  double px, double py,
  const Point2D & a,
  const Point2D & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double length_squared = vx * vx + vy * vy;

  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return distance(px, py, a.x, a.y);
  }

  const double t_unclamped = ((px - a.x) * vx + (py - a.y) * vy) / length_squared;
  const double t = std::clamp(t_unclamped, 0.0, 1.0);
  const double proj_x = a.x + t * vx;
  const double proj_y = a.y + t * vy;
  return distance(px, py, proj_x, proj_y);
}

double distancePointToPolyline(
  double px, double py,
  const std::vector<Point2D> & points)
{
  if (points.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  if (points.size() == 1U) {
    return distance(px, py, points.front().x, points.front().y);
  }

  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i < points.size(); ++i) {
    best = std::min(best, distancePointToSegment(px, py, points[i - 1], points[i]));
  }
  return best;
}

bool validPolyline(const std::vector<Point2D> & points)
{
  return points.size() >= 2U;
}

}  // namespace geometry_utils
}  // namespace semantic_costmap_plugins
