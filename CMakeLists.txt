cmake_minimum_required(VERSION 3.8)
project(f1tenth_control)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# compile_commands.json 생성 활성화 (VS Code Linter 연동용)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# ==============================================================================
# 임베디드 제어 성능 극대화를 위한 컴파일 최적화 플래그 (-O3, -march=native)
# ==============================================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -O3                  # 최대 런타임 속도 최적화
    -march=native        # 구동 중인 하드웨어(Jetson, PC) CPU 명령어셋 맞춤 최적화
    -flto                # 링크 타임 최적화 (Link Time Optimization)
    -Wall -Wextra        # 안전 코딩 규격 경고 활성화
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ==============================================================================
# 1. Autonomous Emergency Braking (AEB) 노드 빌드
# ==============================================================================
add_executable(aeb_node control_code/aeb_node.cpp)
target_include_directories(aeb_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(aeb_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
)

# ==============================================================================
# 2. Xbox Teleop Monitor 노드 빌드
# ==============================================================================
add_executable(joy_teleop_monitor control_code/joy_teleop_monitor.cpp)
target_include_directories(joy_teleop_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joy_teleop_monitor
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
)

# ==============================================================================
# 3. Control MAP 노드 빌드 (L1 Guidance + Steering LUT)
# ==============================================================================
add_executable(control_map_node
  control_code/control_MAP.cpp
  control_code/gap_follower.cpp
  control_code/imu_stability_controller.cpp
)
target_include_directories(control_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_map_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
  f110_msgs
  ament_index_cpp
)

# ==============================================================================
# 4. Teleop Dashboard 뷰어 노드 빌드 (대시보드를 별도 터미널에서 렌더링)
# ==============================================================================
add_executable(teleop_dashboard_node control_code/teleop_dashboard_node.cpp)
ament_target_dependencies(teleop_dashboard_node
  rclcpp
  std_msgs
)

# ==============================================================================
# 5. LUT Calibrator 노드 빌드 (실차 주행 기반 Steering LUT 실측 보정, 관찰 전용)
# ==============================================================================
add_executable(lut_calibrator_node control_code/lut_calibrator_node.cpp)
target_include_directories(lut_calibrator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lut_calibrator_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
  ament_index_cpp
)

# ==============================================================================
# 6. Sim IMU Bridge 노드 빌드 (시뮬 전용 — odom 요레이트 → /imu/data 중계)
# ==============================================================================
add_executable(sim_imu_bridge_node control_code/sim_imu_bridge_node.cpp)
ament_target_dependencies(sim_imu_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# ==============================================================================
# 7. MPPI 컨트롤러 라이브러리 (control_MPPI.cpp — 알고리즘 본체, 컴파일 검증용)
#    샘플링 기반 MPPI(동역학 자전거+Pacejka, 조향+가속 동시). 별도 헤더 없이 단일
#    파일에 인라인 정의(control_MAP.cpp 패턴). 외부 솔버 의존 없음(순수 표준 수학).
#    ⚠️ 다음 세션: 이 파일에 ControlMppiNode + main()을 추가하고 아래를
#       add_library → add_executable(control_mppi_node control_code/control_MPPI.cpp)로
#       전환한 뒤 install(TARGETS ...) 목록에 넣는다.
# ==============================================================================
add_library(control_mppi_solver STATIC
  control_code/control_MPPI.cpp
)

# ==============================================================================
# 8. 파일 설치 (Install) 규칙
# ==============================================================================
install(TARGETS
  aeb_node
  joy_teleop_monitor
  control_map_node
  teleop_dashboard_node
  lut_calibrator_node
  sim_imu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Steering Lookup Table(LUT) CSV — 패키지 share에 설치해 이식성 있는 ament 경로로 로드
# (control_map_node가 get_package_share_directory("f1tenth_control")+"/cfg/..."로 조회)
install(FILES control_code/NUC6_glc_pacejka_lookup_table.csv
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_package()
