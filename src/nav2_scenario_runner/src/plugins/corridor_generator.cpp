// ========================================================================
// 文件: src/nav2_scenario_runner/src/plugins/corridor_generator.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// CorridorGenerator 插件：generate() 根据 seed/waypoint_count 生成狭长通道 waypoint 序列，写入
// ScenarioBundle。请生成类声明和 generate 方法骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include "nav2_scenario_runner/plugins/corridor_generator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_scenario_runner::plugins
{

std::string CorridorGenerator::name() const
{
  return "corridor";
}


  // 读取 generator 插件参数
void CorridorGenerator::configure(const rclcpp::Node::SharedPtr & node)
{
  // 通道宽度、墙厚、keepout 代价都可从 launch 覆盖
  node->declare_parameter("corridor_width", corridor_width_);
  node->declare_parameter("wall_thickness", wall_thickness_);
  node->declare_parameter("keepout_cost", keepout_cost_);
  node->get_parameter("corridor_width", corridor_width_);
  node->get_parameter("wall_thickness", wall_thickness_);
  node->get_parameter("keepout_cost", keepout_cost_);
}

  // 根据 request 生成 walls/waypoints/semantic 区域
ScenarioSpec CorridorGenerator::generate(const ScenarioRequest & request)
{
  ScenarioSpec spec;
  spec.case_id = request.case_id;
  spec.scenario_type = request.scenario_type;
  spec.seed = request.seed;
  // 起点靠地图西侧，沿 x 轴穿通道
  spec.start_pose = Pose2D{-request.map_width / 2.0 + 0.8, 0.0, 0.0};

  const double mw = request.map_width;
  const double mh = request.map_height;
  const double t = wall_thickness_;

  // 四周边界墙，代价 254 相当于硬障碍
  spec.walls.push_back({"north_wall", 0.0, mh / 2.0 - t / 2.0, mw, t, 0.0, 254, true});
  spec.walls.push_back({"south_wall", 0.0, -mh / 2.0 + t / 2.0, mw, t, 0.0, 254, true});
  spec.walls.push_back({"east_wall", mw / 2.0 - t / 2.0, 0.0, t, mh, 0.0, 254, true});
  spec.walls.push_back({"west_wall", -mw / 2.0 + t / 2.0, 0.0, t, mh, 0.0, 254, true});

  const double blocker_h = std::max(1.0, (mh - corridor_width_) * 0.5);
  // 上下两块把地图收成一条水平通道
  spec.static_obstacles.push_back({"upper_block", 0.0, corridor_width_ / 2.0 + blocker_h / 2.0, mw * 0.60, blocker_h, 0.0, 0, true});
  spec.static_obstacles.push_back({"lower_block", 0.0, -corridor_width_ / 2.0 - blocker_h / 2.0, mw * 0.60, blocker_h, 0.0, 0, true});

  if (request.enable_semantic_regions) {
    // LINK[李熠城]：软代价区 + keepout + 偏好车道，供 semantic costmap 联调
    spec.soft_cost_regions.push_back({"door_buffer", -0.5, 0.0, 0.8, corridor_width_ + 0.8, 0.0, 110, true});
    spec.keepout_regions.push_back({"maintenance_zone", mw / 2.5, 0.0, 0.7, 0.7, 0.0, static_cast<int>(keepout_cost_), true});

    PolylineLane lane;
    lane.id = "main_lane";
    lane.points_x = {-mw / 2.0 + 0.8, -1.0, 1.0, mw / 2.0 - 0.8};
    lane.points_y = {0.0, 0.0, 0.0, 0.0};
    lane.corridor_width = corridor_width_;
    lane.inner_penalty = 0.0;
    lane.outside_gain = 20.0;
    lane.max_penalty = 120.0;
    spec.lanes.push_back(lane);
  }

  for (int i = 0; i < request.waypoint_count; ++i) {
    // waypoint 沿通道中心线均匀分布
    const double alpha = static_cast<double>(i + 1) / static_cast<double>(request.waypoint_count + 1);
    spec.waypoints.push_back(Pose2D{-mw / 2.0 + 0.8 + alpha * (mw - 1.6), 0.0, 0.0});
  }

  spec.metadata["generator"] = "CorridorGenerator";
  spec.metadata["description"] = "Single main corridor with optional semantic zones.";
  return spec;
}

}  // namespace nav2_scenario_runner::plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_scenario_runner::plugins::CorridorGenerator,
  nav2_scenario_runner::ScenarioGenerator)
