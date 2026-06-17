// ========================================================================
// 文件: src/semantic_costmap_plugins/src/geometry_utils.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// geometry_utils：world↔map 坐标、点是否在圆/多边形内、栅格索引。请生成纯函数声明和空实现，方便 gtest。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【模块说明】语义 costmap 插件实现，参数见 config/nav2_params_semantic.yaml
// ========================================================================
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
  // 参数里的角度一般是度，内部计算用弧度
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
  // 圆心距离平方比较，避免开方
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

  // 把查询点变换到矩形局部坐标系再判范围
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
  // 射线法判点是否在多边形内，keepout 区域用
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

// 点到线段最短距离，lane penalty 核心
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
  // 到折线各段距离的最小值，偏好车道 penalty 依赖这个
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

// 至少两个点才算折线
bool validPolyline(const std::vector<Point2D> & points)
{
  return points.size() >= 2U;
}

}  // namespace geometry_utils
}  // namespace semantic_costmap_plugins
