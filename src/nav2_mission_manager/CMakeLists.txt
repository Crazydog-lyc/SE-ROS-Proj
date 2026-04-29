cmake_minimum_required(VERSION 3.8)
project(nav2_mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    Dockerfile
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_mission_loader test/test_mission_loader.py)
  ament_add_pytest_test(test_retry_policy test/test_retry_policy.py)
  ament_add_pytest_test(test_state_machine test/test_state_machine.py)
  ament_add_pytest_test(test_navigator_adapter test/test_navigator_adapter.py)
endif()

ament_package()
