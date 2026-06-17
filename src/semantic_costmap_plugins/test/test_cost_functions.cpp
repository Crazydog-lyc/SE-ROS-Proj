// ========================================================================
// 文件: src/semantic_costmap_plugins/test/test_cost_functions.cpp
// 负责人: 李熠城 | 需求: FR-C | PPT: 第17-18页 语义costmap
// ========================================================================
//
// 【AI-PROMPT】
// cost_functions：merge_strategy max/add/replace，把 zone cost 写入 master_grid。请生成函数骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// 【单元测试】几何与代价融合 gtest
// ========================================================================
#include <gtest/gtest.h>

#include "nav2_costmap_2d/cost_values.hpp"
#include "semantic_costmap_plugins/cost_functions.hpp"

namespace cf = semantic_costmap_plugins::cost_functions;
using semantic_costmap_plugins::MergeStrategy;

// merge/max/add 行为
TEST(CostFunctions, ModeMatching)
{
  EXPECT_TRUE(cf::modeMatches("all", "patrol"));
  EXPECT_TRUE(cf::modeMatches("*", "charge"));
  EXPECT_TRUE(cf::modeMatches("delivery", "delivery"));
  EXPECT_FALSE(cf::modeMatches("patrol", "charge"));
}

// merge/max/add 行为
TEST(CostFunctions, ParseMergeStrategy)
{
  EXPECT_EQ(cf::parseMergeStrategy("max"), MergeStrategy::Max);
  EXPECT_EQ(cf::parseMergeStrategy("add"), MergeStrategy::Add);
  EXPECT_EQ(cf::parseMergeStrategy("overwrite"), MergeStrategy::Overwrite);
}

// merge/max/add 行为
TEST(CostFunctions, MergeCostsMax)
{
  EXPECT_EQ(cf::mergeCosts(10U, 20U, MergeStrategy::Max), 20U);
  EXPECT_EQ(cf::mergeCosts(50U, 5U, MergeStrategy::Max), 50U);
}

// merge/max/add 行为
TEST(CostFunctions, MergeCostsAdd)
{
  EXPECT_EQ(cf::mergeCosts(10U, 20U, MergeStrategy::Add), 30U);
  EXPECT_EQ(
    cf::mergeCosts(250U, 20U, MergeStrategy::Add),

    static_cast<unsigned char>(nav2_costmap_2d::LETHAL_OBSTACLE - 1U));
}

// merge/max/add 行为
TEST(CostFunctions, MergeCostsOverwrite)
{
  EXPECT_EQ(cf::mergeCosts(10U, 20U, MergeStrategy::Overwrite), 20U);
}

// merge/max/add 行为
TEST(CostFunctions, LanePenalty)
{
  EXPECT_EQ(cf::lanePenalty(0.0, 1.0, 0.0, 20.0, 100.0), 0U);
  EXPECT_GT(cf::lanePenalty(1.0, 1.0, 0.0, 20.0, 100.0), 0U);
  EXPECT_EQ(cf::lanePenalty(10.0, 1.0, 0.0, 20.0, 80.0), 80U);
}

// merge/max/add 行为
TEST(CostFunctions, CongestionPenalty)
{
  EXPECT_EQ(cf::congestionPenalty(2.0, 1.0, 120.0, 1.0), 0U);
  EXPECT_GT(cf::congestionPenalty(0.1, 1.0, 120.0, 1.0), 0U);
  EXPECT_GT(cf::congestionPenalty(0.0, 1.0, 120.0, 2.0), 100U);
}
