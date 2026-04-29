cmake_minimum_required(VERSION 3.8)
project(sam_bot_safety_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(course_interfaces REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/follow_waypoints_demo.py
  scripts/follow_waypoints_demo_blockage_recovery.py
  scripts/reach_goal_demo_failure.py
  scripts/reach_goal_demo_pause_near_obstacle.py
  scripts/reach_goal_demo_success.py
  scripts/safety_demo_console.py
  scripts/safety_demo_injector.py
  scripts/safety_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
