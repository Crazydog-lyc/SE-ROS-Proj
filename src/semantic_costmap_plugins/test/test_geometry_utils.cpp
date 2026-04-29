#include <gtest/gtest.h>

#include "semantic_costmap_plugins/geometry_utils.hpp"

using semantic_costmap_plugins::Point2D;
namespace gu = semantic_costmap_plugins::geometry_utils;

TEST(GeometryUtils, PointInsideCircle)
{
  EXPECT_TRUE(gu::pointInCircle(0.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_TRUE(gu::pointInCircle(0.5, 0.5, 0.0, 0.0, 1.0));
  EXPECT_FALSE(gu::pointInCircle(1.1, 0.0, 0.0, 0.0, 1.0));
}

TEST(GeometryUtils, PointInsideRotatedRectangle)
{
  EXPECT_TRUE(gu::pointInRotatedRectangle(0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0));
  EXPECT_FALSE(gu::pointInRotatedRectangle(2.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0));
  EXPECT_TRUE(gu::pointInRotatedRectangle(0.2, 0.2, 0.0, 0.0, 2.0, 2.0, gu::deg2rad(45.0)));
}

TEST(GeometryUtils, PointInsidePolygon)
{
  std::vector<Point2D> polygon{
    {0.0, 0.0},
    {2.0, 0.0},
    {2.0, 2.0},
    {0.0, 2.0}
  };

  EXPECT_TRUE(gu::pointInPolygon(1.0, 1.0, polygon));
  EXPECT_FALSE(gu::pointInPolygon(3.0, 1.0, polygon));
}

TEST(GeometryUtils, DistancePointToSegment)
{
  Point2D a{0.0, 0.0};
  Point2D b{2.0, 0.0};

  EXPECT_NEAR(gu::distancePointToSegment(1.0, 1.0, a, b), 1.0, 1e-6);
  EXPECT_NEAR(gu::distancePointToSegment(-1.0, 0.0, a, b), 1.0, 1e-6);
  EXPECT_NEAR(gu::distancePointToSegment(1.0, 0.0, a, b), 0.0, 1e-6);
}

TEST(GeometryUtils, DistancePointToPolyline)
{
  std::vector<Point2D> line{
    {0.0, 0.0},
    {2.0, 0.0},
    {2.0, 2.0}
  };

  EXPECT_NEAR(gu::distancePointToPolyline(1.0, 1.0, line), 1.0, 1e-6);
  EXPECT_NEAR(gu::distancePointToPolyline(2.0, 1.0, line), 0.0, 1e-6);
  EXPECT_TRUE(gu::validPolyline(line));
  EXPECT_FALSE(gu::validPolyline({}));
}
