// ========================================================================
// 文件: src/nav2_scenario_runner/include/nav2_scenario_runner/scenario_types.hpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// 帮我 scaffold 一个 nav2_scenario_runner C++ 包：pluginlib 注册 ScenarioGenerator 插件，包含
// generator_registry、scenario_types、scenario_serializer，再写 generate_scenario_node 和 Python
// 侧 generate_cases/run_batch 脚本入口。要求固定 random seed、每个 case 输出独立目录；具体 corridor/room
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#pragma once

#include <map>
#include <string>
#include <vector>

namespace nav2_scenario_runner
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RectRegion
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double width{1.0};
  double height{1.0};
  double yaw{0.0};

  int cost{0};
  bool enabled{true};
};

struct PolylineLane
{
  std::string id;
  std::vector<double> points_x;
  std::vector<double> points_y;
  double corridor_width{1.0};
  double inner_penalty{0.0};
  double outside_gain{15.0};
  double max_penalty{120.0};
  bool enabled{true};
};

struct DynamicEvent
{
  std::string type;
  double trigger_time{0.0};
  double duration{0.0};
  RectRegion region;
  double strength{0.0};
};

struct ScenarioRequest
{
  std::string case_id{"case_000"};
  std::string scenario_type{"corridor"};
  int seed{42};
  int waypoint_count{4};
  int room_count{3};
  int obstacle_density{2};
  bool enable_semantic_regions{true};
  bool enable_faults{false};
  bool enable_congestion{false};
  double map_width{8.0};
  double map_height{8.0};
};

struct ScenarioSpec
{
  std::string case_id;
  std::string scenario_type;
  int seed{42};
  Pose2D start_pose;
  std::vector<RectRegion> walls;
  std::vector<RectRegion> static_obstacles;
  std::vector<RectRegion> keepout_regions;
  std::vector<RectRegion> soft_cost_regions;
  std::vector<PolylineLane> lanes;
  std::vector<Pose2D> waypoints;
  std::vector<DynamicEvent> events;
  std::map<std::string, std::string> metadata;
};

}  // namespace nav2_scenario_runner
