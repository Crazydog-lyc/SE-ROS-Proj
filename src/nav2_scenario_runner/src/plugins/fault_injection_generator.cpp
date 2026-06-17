// ========================================================================
// 文件: src/nav2_scenario_runner/src/plugins/fault_injection_generator.cpp
// 负责人: 陆华均 | 需求: FR-A | PPT: 第21-22页 场景生成
// ========================================================================
//
// 【AI-PROMPT】
// FaultInjectionGenerator：在 case manifest 里写入 sensor_timeout/tf_fault 等故障注入参数，供
// safety_monitor Demo 用。请生成插件骨架。
//
// 【AI-SCOPE】import · declare · register · 插件/接口空壳
// ========================================================================
#include "nav2_scenario_runner/plugins/fault_injection_generator.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace nav2_scenario_runner::plugins
{

std::string FaultInjectionGenerator::name() const
{
  return "fault_injection";
}


  // 读取 generator 插件参数
void FaultInjectionGenerator::configure(const rclcpp::Node::SharedPtr & node)
// TODO[陆华均]：FR-A-01 【FR-05】FaultInjectionGenerator：传感器/TF 故障注入参数
{
  node->declare_parameter("fault_trigger_time", fault_trigger_time_);
  node->declare_parameter("fault_duration", fault_duration_);
  node->get_parameter("fault_trigger_time", fault_trigger_time_);
  node->get_parameter("fault_duration", fault_duration_);
}

// 带 sensor fault 标记的 case
ScenarioSpec FaultInjectionGenerator::generate(const ScenarioRequest & request)
{
  ScenarioSpec spec;
  spec.case_id = request.case_id;
  spec.scenario_type = request.scenario_type;
  spec.seed = request.seed;
  spec.start_pose = Pose2D{-2.5, -2.5, 0.0};

  spec.walls.push_back({"north_wall", 0.0, 3.0, 7.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"south_wall", 0.0, -3.0, 7.0, 0.2, 0.0, 254, true});
  spec.walls.push_back({"east_wall", 3.4, 0.0, 0.2, 6.0, 0.0, 254, true});
  spec.walls.push_back({"west_wall", -3.4, 0.0, 0.2, 6.0, 0.0, 254, true});

  spec.static_obstacles.push_back({"storage", -0.5, 0.8, 1.0, 1.6, 0.0, 0, true});
  spec.waypoints = {{-2.5, -2.5, 0.0}, {2.6, -2.5, 0.0}, {2.6, 2.2, 1.57}};

  if (request.enable_faults) {
    DynamicEvent pause_event;
    pause_event.type = "pause";
    pause_event.trigger_time = fault_trigger_time_;
    pause_event.duration = fault_duration_;
    pause_event.region = RectRegion{"fault_scope", 0.0, 0.0, 0.1, 0.1, 0.0, 0, true};
    pause_event.strength = 1.0;
    spec.events.push_back(pause_event);

    DynamicEvent tf_event;
    tf_event.type = "tf_fault";
    tf_event.trigger_time = fault_trigger_time_ + 5.0;
    tf_event.duration = 3.0;
    tf_event.region = RectRegion{"tf_scope", 0.0, 0.0, 0.1, 0.1, 0.0, 0, true};
    tf_event.strength = 1.0;
    spec.events.push_back(tf_event);
  }

  spec.metadata["generator"] = "FaultInjectionGenerator";
  spec.metadata["description"] = "Scenario with scheduled pause and TF fault injection markers.";
  return spec;
}

}  // namespace nav2_scenario_runner::plugins

PLUGINLIB_EXPORT_CLASS(
  nav2_scenario_runner::plugins::FaultInjectionGenerator,
  nav2_scenario_runner::ScenarioGenerator)
