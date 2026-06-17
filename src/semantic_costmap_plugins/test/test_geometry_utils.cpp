// ========================================================================
// 文件: src/semantic_costmap_plugins/test/test_geometry_utils.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// geometry_utils：world↔map 坐标、点是否在圆/多边形内、栅格索引。请生成纯函数声明和空实现，方便 gtest。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【单元测试】几何与代价融合 gtest
// ========================================================================
#include <gtest/gtest.h>

#include "semantic_costmap_plugins/geometry_utils.hpp"

using semantic_costmap_plugins::Point2D;
namespace gu = semantic_costmap_plugins::geometry_utils;

// 几何工具 gtest
TEST(GeometryUtils, PointInsideCircle)
{
  EXPECT_TRUE(gu::pointInCircle(0.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_TRUE(gu::pointInCircle(0.5, 0.5, 0.0, 0.0, 1.0));
  EXPECT_FALSE(gu::pointInCircle(1.1, 0.0, 0.0, 0.0, 1.0));
}

// 几何工具 gtest
TEST(GeometryUtils, PointInsideRotatedRectangle)
{
  EXPECT_TRUE(gu::pointInRotatedRectangle(0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0));
  EXPECT_FALSE(gu::pointInRotatedRectangle(2.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0));
  EXPECT_TRUE(gu::pointInRotatedRectangle(0.2, 0.2, 0.0, 0.0, 2.0, 2.0, gu::deg2rad(45.0)));
}


// 几何工具 gtest
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

// 几何工具 gtest
TEST(GeometryUtils, DistancePointToSegment)
{
  Point2D a{0.0, 0.0};
  Point2D b{2.0, 0.0};

  EXPECT_NEAR(gu::distancePointToSegment(1.0, 1.0, a, b), 1.0, 1e-6);
  EXPECT_NEAR(gu::distancePointToSegment(-1.0, 0.0, a, b), 1.0, 1e-6);
  EXPECT_NEAR(gu::distancePointToSegment(1.0, 0.0, a, b), 0.0, 1e-6);
}

// 几何工具 gtest
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
