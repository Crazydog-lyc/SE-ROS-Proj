cmake_minimum_required(VERSION 3.8)
project(nav2_scenario_runner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core SHARED
  src/generator_registry.cpp
  src/scenario_serializer.cpp
  src/plugins/corridor_generator.cpp
  src/plugins/room_inspection_generator.cpp
  src/plugins/congestion_generator.cpp
  src/plugins/fault_injection_generator.cpp
)

ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  pluginlib
  geometry_msgs
  std_msgs
  nav_msgs
)

target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_scenario_runner plugins.xml)

add_executable(generate_scenario_node src/generate_scenario_node.cpp)
ament_target_dependencies(generate_scenario_node rclcpp pluginlib geometry_msgs std_msgs nav_msgs)
target_link_libraries(generate_scenario_node ${PROJECT_NAME}_core yaml-cpp)

install(
  TARGETS ${PROJECT_NAME}_core generate_scenario_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/generate_cases.py
    scripts/run_batch.py
    scripts/summarize_results.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_case_matrix_builder
    test/test_case_matrix_builder.cpp
    src/scenario_serializer.cpp
  )
  if(TARGET test_case_matrix_builder)
    ament_target_dependencies(test_case_matrix_builder rclcpp pluginlib geometry_msgs std_msgs nav_msgs)
    target_link_libraries(test_case_matrix_builder yaml-cpp)
    target_include_directories(test_case_matrix_builder PUBLIC include)
  endif()

  ament_add_gtest(test_metrics_utils test/test_metrics_utils.cpp)
  if(TARGET test_metrics_utils)
    target_include_directories(test_metrics_utils PUBLIC include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp pluginlib geometry_msgs std_msgs nav_msgs)
ament_package()
