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
# 3. Steering Control 노드 빌드
# ==============================================================================
add_executable(steering_control_node 
  control_code/steering_control_node.cpp
  control_code/geometry_utils.cpp
  control_code/gap_follower.cpp
  control_code/velocity_profiler.cpp
  control_code/stability_controller.cpp
)
target_include_directories(steering_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(steering_control_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
)

# ==============================================================================
# 4. 파일 설치 (Install) 규칙
# ==============================================================================
install(TARGETS
  aeb_node
  joy_teleop_monitor
  steering_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
