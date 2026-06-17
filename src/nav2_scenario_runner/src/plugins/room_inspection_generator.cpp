// ========================================================================
// 文件: src/nav2_scenario_runner/src/plugins/room_inspection_generator.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// RoomInspectionGenerator：在矩形房间内生成网格巡检路径 waypoint，支持 seed 复现。请生成 plugin 类骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include "nav2_scenario_runner/plugins/room_inspection_generator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_scenario_runner::plugins
{

std::string RoomInspectionGenerator::name() const
{
  return "room_inspection";
}


// 房间数、障碍密度参数
void RoomInspectionGenerator::configure(const rclcpp::Node::SharedPtr & node)
// TODO[陆华均]：FR-A-01 RoomInspectionGenerator：房间巡检路径
{
  node->declare_parameter("room_size", room_size_);
  node->declare_parameter("doorway_width", doorway_width_);
  node->get_parameter("room_size", room_size_);
  node->get_parameter("doorway_width", doorway_width_);
}

// 多房间巡检 waypoint
ScenarioSpec RoomInspectionGenerator::generate(const ScenarioRequest & request)
{
  ScenarioSpec spec;
  spec.case_id = request.case_id;
  spec.scenario_type = request.scenario_type;
  spec.seed = request.seed;
  spec.start_pose = Pose2D{-3.2, -2.6, 0.0};

  spec.walls.push_back({"north_wall", 0.0, 3.9, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"south_wall", 0.0, -3.9, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"east_wall", 3.9, 0.0, 0.2, 8.0, 0.0, 254, true});
  spec.walls.push_back({"west_wall", -3.9, 0.0, 0.2, 8.0, 0.0, 254, true});

  spec.static_obstacles.push_back({"left_partition", -1.2, 0.0, 0.2, 5.8, 0.0, 0, true});
  spec.static_obstacles.push_back({"right_partition", 1.2, 0.0, 0.2, 5.8, 0.0, 0, true});
  spec.static_obstacles.push_back({"top_partition", 0.0, 1.2, 6.2, 0.2, 0.0, 0, true});

  if (request.enable_semantic_regions) {
    spec.keepout_regions.push_back({"server_room", 2.5, -2.4, 0.9, 0.9, 0.0, 254, true});
    spec.soft_cost_regions.push_back({"inspection_buffer_a", -2.5, 2.2, 1.1, 1.1, 0.0, 120, true});
    spec.soft_cost_regions.push_back({"inspection_buffer_b", 0.0, -2.0, 1.1, 1.1, 0.0, 100, true});
  }

  spec.waypoints = {
    {-2.6, -2.5, 0.0},
    {-2.6, 2.3, 1.57},
    {0.0, 2.3, 0.0},
    {2.6, 2.3, 0.0},
    {2.6, -2.0, -1.57},
    {0.0, -2.6, 3.14}
  };

  spec.metadata["generator"] = "RoomInspectionGenerator";
  spec.metadata["description"] = "Multi-room inspection route with semantic keepout areas.";
  return spec;
}

}  // namespace nav2_scenario_runner::plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_scenario_runner::plugins::RoomInspectionGenerator,
  nav2_scenario_runner::ScenarioGenerator)
