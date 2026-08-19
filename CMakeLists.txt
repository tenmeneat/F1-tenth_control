cmake_minimum_required(VERSION 3.8)
project(f1tenth_control)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# compile_commands.json 생성 활성화 (VS Code Linter 연동용)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    "$<$<COMPILE_LANGUAGE:CXX>:-O3>"          # 최대 런타임 속도 최적화
    "$<$<COMPILE_LANGUAGE:CXX>:-march=native>" # 구동 하드웨어(Jetson, PC) CPU 명령어셋 맞춤
    "$<$<COMPILE_LANGUAGE:CXX>:-flto>"         # 링크 타임 최적화 (Link Time Optimization)
    "$<$<COMPILE_LANGUAGE:CXX>:-Wall>"         # 안전 코딩 규격 경고 활성화
    "$<$<COMPILE_LANGUAGE:CXX>:-Wextra>"
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ==============================================================================
# 2. Control MAP 노드 빌드 (L1 Guidance + 정상상태 자전거 역모델 조향)
# ==============================================================================
add_executable(control_map_node
  control_code/control_map_node.cpp
)
target_include_directories(control_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(control_map_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${f110_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
)

# ==============================================================================
# 5. Sim IMU Bridge 노드 빌드 (시뮬 전용 — odom 요레이트 → /imu/data 중계)
# ==============================================================================
add_executable(sim_imu_bridge_node control_code/sim_imu_bridge_node.cpp)
target_link_libraries(sim_imu_bridge_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(drive_source_selector control_code/drive_source_selector.cpp)
target_link_libraries(drive_source_selector
  rclcpp::rclcpp
  ${ackermann_msgs_TARGETS}
)

add_executable(odom_calib_node control_code/odom_calib_node.cpp)
target_link_libraries(odom_calib_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
)

# ==============================================================================
# 8. 파일 설치 (Install) 규칙
# ==============================================================================
install(TARGETS
  drive_source_selector
  odom_calib_node
  control_map_node
  sim_imu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 🟢 2026-08-19: config/sectors.yaml 삭제. sector_learner가 /global_waypoints의 κ로
#    섹터 경계를 그 자리에서 만들므로(autobuild, 08-13) 씨앗 파일이 필요 없어졌다.
#    표를 직접 주고 싶으면 `sector_scale_file:=<경로>` 하나로 넘긴다(학습 결과는
#    learned/ 에 저장된다). 자세한 배경은 CLAUDE.md ②-x.

install(PROGRAMS scripts/sector_learner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
