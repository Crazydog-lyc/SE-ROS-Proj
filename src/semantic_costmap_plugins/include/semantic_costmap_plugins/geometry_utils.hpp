#ifndef SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_
#define SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_

#include <vector>

#include "semantic_costmap_plugins/semantic_types.hpp"

namespace semantic_costmap_plugins
{
namespace geometry_utils
{

double deg2rad(double degrees);  // 度转弧度
double distance(double x1, double y1, double x2, double y2);  // 平面欧氏距离
double distanceSquared(double x1, double y1, double x2, double y2);  // 距离平方，避免开方

Point2D rotatePointAroundOrigin(const Point2D & point, double yaw_rad);  // 绕原点旋转
Point2D translatePoint(const Point2D & point, double tx, double ty);  // 平移点

bool pointInCircle(  // 点是否在圆内
  double px, double py,
  double cx, double cy,
  double radius);

bool pointInRotatedRectangle(  // 点是否在旋转矩形内
  double px, double py,
  double cx, double cy,
  double width, double height,
  double yaw_rad);

bool pointInPolygon(  // 射线法判多边形
  double px, double py,
  const std::vector<Point2D> & polygon);

double distancePointToSegment(  // 点到线段距离
  double px, double py,
  const Point2D & a,
  const Point2D & b);

double distancePointToPolyline(  // 点到折线最短距离
  double px, double py,
  const std::vector<Point2D> & points);

bool validPolyline(const std::vector<Point2D> & points);  // 至少两个顶点

}  // namespace geometry_utils
}  // namespace semantic_costmap_plugins

#endif  // SEMANTIC_COSTMAP_PLUGINS__GEOMETRY_UTILS_HPP_
