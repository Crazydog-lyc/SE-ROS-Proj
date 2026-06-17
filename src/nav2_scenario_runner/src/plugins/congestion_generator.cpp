// ========================================================================
// 文件: src/nav2_scenario_runner/src/plugins/congestion_generator.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// CongestionGenerator：生成带临时高代价区的 semantic_overlay.yaml，waypoint 穿过拥堵区。请生成插件骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include "nav2_scenario_runner/plugins/congestion_generator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_scenario_runner::plugins
{

std::string CongestionGenerator::name() const
{
  return "congestion";
}


  // 读取 generator 插件参数
void CongestionGenerator::configure(const rclcpp::Node::SharedPtr & node)
{
  // 动态拥堵事件的默认触发时刻和强度
  node->declare_parameter("default_trigger_time", default_trigger_time_);
  node->declare_parameter("default_duration", default_duration_);
  node->declare_parameter("default_strength", default_strength_);
  node->get_parameter("default_trigger_time", default_trigger_time_);
  node->get_parameter("default_duration", default_duration_);
  node->get_parameter("default_strength", default_strength_);
}

  // 根据 request 生成 walls/waypoints/semantic 区域
ScenarioSpec CongestionGenerator::generate(const ScenarioRequest & request)
{
  ScenarioSpec spec;
  spec.case_id = request.case_id;
  spec.scenario_type = request.scenario_type;
  spec.seed = request.seed;
  spec.start_pose = Pose2D{-3.0, 0.0, 0.0};

  spec.walls.push_back({"north_wall", 0.0, 3.0, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"south_wall", 0.0, -3.0, 8.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"east_wall", 3.9, 0.0, 0.2, 6.0, 0.0, 254, true});
  spec.walls.push_back({"west_wall", -3.9, 0.0, 0.2, 6.0, 0.0, 254, true});

  spec.static_obstacles.push_back({"center_island", 0.0, 0.0, 1.2, 1.2, 0.0, 0, true});
  spec.waypoints = {{-2.8, 0.0, 0.0}, {2.8, 0.0, 0.0}};

  if (request.enable_congestion) {
    // LINK[李熠城]：events 会进 overlay，runtime 由 publish_sample_congestion 模拟
    DynamicEvent event;
    event.type = "congestion";
    event.trigger_time = default_trigger_time_;
    event.duration = default_duration_;
    event.region = RectRegion{"hotspot", 1.2, 0.0, 1.0, 1.6, 0.0, 0, true};
    event.strength = default_strength_;
    spec.events.push_back(event);
  }

  spec.metadata["generator"] = "CongestionGenerator";
  spec.metadata["description"] = "Simple point-to-point scenario with online congestion event.";
  return spec;
}

}  // namespace nav2_scenario_runner::plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_scenario_runner::plugins::CongestionGenerator,
  nav2_scenario_runner::ScenarioGenerator)
