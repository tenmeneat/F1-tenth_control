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
  # ⚠️ CXX 전용으로 스코프 제한 — nvcc(CUDA)는 -march=native/-flto를 직접 못 알아들어
  #    CUDA 활성화 시 .cu 컴파일이 실패한다. .cu용 최적화는 아래 CUDA 블록에서 별도 지정.
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
  control_code/control_map_node.cpp
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
# 7. MPPI 컨트롤러 노드 (control_mppi_node) — control_map_node와 동급 상시 구동
#    샘플링 기반 MPPI(동역학 자전거+Pacejka, 조향+가속 동시)로 글로벌 경로 추종,
#    결과를 /drive_mppi로 발행. Mux(joy_teleop_monitor)가 RB로 MAP↔MPPI 라우팅.
#    솔버는 컴파일 타임 자동선택: CUDA 있으면 GPU(아래 7-B에서 USE_MPPI_GPU 정의 +
#    control_mppi_gpu_solver 링크), 없으면 CPU(control_mppi_node.cpp가
#    control_mppi_solver_cpu.cpp를 직접 include). 어느 쪽이든 실행파일은 항상 빌드된다.
# ==============================================================================
add_executable(control_mppi_node
  control_code/control_mppi_node.cpp
)
target_include_directories(control_mppi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# CPU 경로에서 control_mppi_node.cpp가 #include "control_mppi_solver_cpu.cpp" 하므로
# control_code를 include 경로에 추가(GPU 경로에선 mppi_gpu.hpp만 쓰므로 무해).
target_include_directories(control_mppi_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/control_code
)
ament_target_dependencies(control_mppi_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  std_msgs
  f110_msgs
  ament_index_cpp
)

# ==============================================================================
# 7-B. MPPI GPU 솔버 (CUDA, 선택적 빌드)
#   control_mppi_solver_gpu.cu — control_mppi_solver_cpu.cpp(CPU)의 롤아웃 코어를 CUDA로 포팅한 것.
#   ⚠️ CUDA 미설치 환경(팀원 PC, CI)에서도 colcon build가 깨지지 않도록 check_language로
#      감지 후에만 활성화. 기존 6개 실행파일/CPU 정적라이브러리는 CUDA 유무와 무관하게 빌드됨.
#   Jetson Orin Nano(sm_87) + 개발PC RTX 4060(sm_89) 포함, 합리적 하위호환 커버리지.
#   cmake 3.22엔 "native" 아키텍처 키워드(3.24+ 전용)가 없어 명시적 목록을 쓴다.
# ==============================================================================
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)  # Thrust 헤더도 CUDAToolkit include 경로로 딸려옴

  if(NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  endif()
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES "75;80;86;87;89")
  endif()

  add_library(control_mppi_gpu_solver STATIC
    control_code/control_mppi_solver_gpu.cu
  )
  target_include_directories(control_mppi_gpu_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(control_mppi_gpu_solver PUBLIC CUDA::cudart)

  # control_mppi_node를 GPU 솔버로 전환: USE_MPPI_GPU 정의 + GPU 솔버/런타임 링크.
  # (CUDA 없으면 이 블록이 스킵돼 노드는 CPU 솔버로 그대로 빌드됨.)
  # ⚠️ ament_target_dependencies가 이미 plain-signature target_link_libraries를 이 타겟에
  #    썼으므로 여기서도 plain(키워드 PRIVATE 없이)으로 맞춰야 함(혼용 시 CMake 에러).
  target_compile_definitions(control_mppi_node PRIVATE USE_MPPI_GPU)
  target_link_libraries(control_mppi_node control_mppi_gpu_solver CUDA::cudart)

  install(TARGETS control_mppi_gpu_solver
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "f1tenth_control: CUDA 컴파일러 없음 — GPU MPPI 솔버 빌드 스킵(CPU 타겟 영향 없음)")
endif()

# ==============================================================================
# 8. 파일 설치 (Install) 규칙
# ==============================================================================
install(TARGETS
  aeb_node
  joy_teleop_monitor
  control_map_node
  control_mppi_node
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
