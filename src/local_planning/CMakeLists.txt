cmake_minimum_required(VERSION 3.8)
project(local_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

include_directories(
  include
)

add_library(local_planner_core STATIC
  src/local_planner_node.cpp
  src/obstacle_guard.cpp
  src/p3_maneuver_lifecycle.cpp
  src/p3_shadow.cpp
  src/raceline_spline_planner.cpp
  src/safe_stop_lifecycle.cpp
)
ament_target_dependencies(local_planner_core
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  f110_msgs
)

add_executable(local_planner_node src/local_planner_main.cpp)
ament_target_dependencies(local_planner_node rclcpp)
target_link_libraries(local_planner_node local_planner_core)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # 진단 하니스가 운영 YAML과 다른 마진으로 조용히 답하는 것을 막는다.
  # (2026-08-14: 실제로 6개 값이 어긋난 채 탈출 임계를 재고 운영값을 정했다.)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(params_match_yaml test/test_params_match_yaml.py)

  # Node-level: covers the subscription callbacks themselves (which arrays/references the node
  # accepts as authoritative). The algorithm gtests cannot reach that wiring.
  ament_add_gtest(test_local_planner_node test/test_local_planner_node.cpp TIMEOUT 120)
  if(TARGET test_local_planner_node)
    ament_target_dependencies(test_local_planner_node
      rclcpp
      std_msgs
      nav_msgs
      f110_msgs
    )
    target_link_libraries(test_local_planner_node local_planner_core)
  endif()

  ament_add_gtest(test_raceline_spline test/test_raceline_spline.cpp)
  if(TARGET test_raceline_spline)
    ament_target_dependencies(test_raceline_spline
      rclcpp
      nav_msgs
      f110_msgs
    )
    target_link_libraries(test_raceline_spline local_planner_core)
  endif()

  ament_add_gtest(test_obstacle_guard test/test_obstacle_guard.cpp)
  if(TARGET test_obstacle_guard)
    ament_target_dependencies(test_obstacle_guard
      f110_msgs
    )
    target_link_libraries(test_obstacle_guard local_planner_core)
  endif()

  ament_add_gtest(test_safe_stop_lifecycle test/test_safe_stop_lifecycle.cpp)
  if(TARGET test_safe_stop_lifecycle)
    target_link_libraries(test_safe_stop_lifecycle local_planner_core)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_p3_maneuver_lifecycle.cpp")
    ament_add_gtest(
      test_p3_maneuver_lifecycle
      test/test_p3_maneuver_lifecycle.cpp)
    if(TARGET test_p3_maneuver_lifecycle)
      ament_target_dependencies(test_p3_maneuver_lifecycle f110_msgs)
      target_link_libraries(test_p3_maneuver_lifecycle local_planner_core)
    endif()
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_p3_production_parity.cpp")
    ament_add_gtest(test_p3_production_parity test/test_p3_production_parity.cpp)
    if(TARGET test_p3_production_parity)
      ament_target_dependencies(test_p3_production_parity f110_msgs)
      target_link_libraries(test_p3_production_parity local_planner_core)
    endif()
  endif()

  add_executable(
    p3_production_parity_harness
    test/p3_production_parity_harness.cpp)
  ament_target_dependencies(p3_production_parity_harness f110_msgs)
  target_link_libraries(p3_production_parity_harness local_planner_core)

  # 실차 정지 교착 재현용 진단 하니스 (런타임 노드 아님)
  add_executable(
    stuck_case_harness
    test/stuck_case_harness.cpp)
  ament_target_dependencies(stuck_case_harness f110_msgs)
  target_link_libraries(stuck_case_harness local_planner_core)

endif()

ament_package()
