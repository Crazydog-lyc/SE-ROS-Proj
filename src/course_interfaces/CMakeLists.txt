cmake_minimum_required(VERSION 3.8)
project(course_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunMission.action"
  "msg/BatchTestResult.msg"
  "msg/MissionState.msg"
  "msg/SafetyState.msg"
  "msg/ZoneRule.msg"
  "msg/ZoneRuleArray.msg"
  "srv/GetSafetyState.srv"
  "srv/LoadScenario.srv"
  "srv/RequestRecovery.srv"
  "srv/ReloadZones.srv"
  "srv/TriggerCancel.srv"
  "srv/TriggerEmergencyStop.srv"
  "srv/TriggerPause.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
