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
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ==============================================================================
# 1. Xbox Teleop Monitor 노드 빌드
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
# 2. Control MAP 노드 빌드 (L1 Guidance + Steering LUT)
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
  visualization_msgs
  ament_index_cpp
)

# ==============================================================================
# 3. Teleop Dashboard 뷰어 노드 빌드 (대시보드를 별도 터미널에서 렌더링)
# ==============================================================================
add_executable(teleop_dashboard_node control_code/teleop_dashboard_node.cpp)
ament_target_dependencies(teleop_dashboard_node
  rclcpp
  std_msgs
)

# ==============================================================================
# 4. LUT Calibrator 노드 빌드 (실차 주행 기반 Steering LUT 실측 보정, 관찰 전용)
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
# 5. Sim IMU Bridge 노드 빌드 (시뮬 전용 — odom 요레이트 → /imu/data 중계)
# ==============================================================================
add_executable(sim_imu_bridge_node control_code/sim_imu_bridge_node.cpp)
ament_target_dependencies(sim_imu_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
)

# ==============================================================================
# 6. MPPI 컨트롤러 노드 (control_mppi_node) — control_map_node와 동급 상시 구동
#    샘플링 기반 MPPI(동역학 자전거+Pacejka, 조향+가속 동시)로 글로벌 경로 추종,
#    결과를 /drive_mppi로 발행. Mux(joy_teleop_monitor)가 RB로 MAP↔MPPI 라우팅.
#    솔버는 컴파일 타임 자동선택: CUDA 있으면 GPU(아래 6-B에서 USE_MPPI_GPU 정의 +
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
# 6-B. MPPI GPU 솔버 (CUDA, 선택적 빌드)
#   control_mppi_solver_gpu.cu — control_mppi_solver_cpu.cpp(CPU)의 롤아웃 코어를 CUDA로
#   포팅한 것.
#   ⚠️ CUDA 미설치 환경(팀원 PC, CI)에서도 colcon build가 깨지지 않도록 check_language로
#      감지 후에만 활성화. 나머지 실행파일/CPU 경로는 CUDA 유무와 무관하게 빌드됨.
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
# 7. Drive Source Selector 노드 (실차 전용 MAP/MPPI 슬림 셀렉터)
#    실차는 수동/자율/E-stop Mux를 f1tenth_stack(drive_mode_manager+ackermann_mux)이 맡고,
#    joy_teleop_monitor는 실차 런치에서 제외된다. 그 스택엔 MAP/MPPI 개념이 없으므로 이 노드가
#    RB로 /drive_autonomous(MAP)↔/drive_mppi(MPPI)를 골라 /drive(navigation 채널)로 포워딩한다.
# ==============================================================================
add_executable(drive_source_selector control_code/drive_source_selector.cpp)
ament_target_dependencies(drive_source_selector
  rclcpp
  sensor_msgs
  ackermann_msgs
  std_msgs
)

# ==============================================================================
# 7-B. Real-car Dashboard 노드 (실차 젯슨 원격 모니터링 — 우리 컴에서 실행)
#    젯슨 원시 토픽(/drive_mode,/mppi_active,/drive,odom,/joy)을 직접 구독해 로컬에서
#    렌더링(젯슨 렌더 연산 0). 시뮬용 teleop_dashboard_node(문자열 뷰어)와 별개 노드.
# ==============================================================================
add_executable(realcar_dashboard_node control_code/realcar_dashboard_node.cpp)
ament_target_dependencies(realcar_dashboard_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
)

# ==============================================================================
# 7-C. Odom 거리 스케일 보정 노드 (실차 원격 — 우리 컴에서 실행)
#    "명령 주고 자로 재기" 테스트 자동화. odom 한 토픽 안의 twist(VESC 휠 패스스루)와
#    pose(MCL 스캔매칭)를 따로 적분해 명령/휠/맵 세 거리를 동시 측정 → 어느 게인이
#    틀렸는지 분리해준다. /drive 미발행(순수 관찰자).
# ==============================================================================
add_executable(odom_calib_node control_code/odom_calib_node.cpp)
ament_target_dependencies(odom_calib_node
  rclcpp
  std_msgs
  nav_msgs
  ackermann_msgs
)

# ==============================================================================
# 8. 파일 설치 (Install) 규칙
# ==============================================================================
install(TARGETS
  joy_teleop_monitor
  drive_source_selector
  realcar_dashboard_node
  odom_calib_node
  control_map_node
  control_mppi_node
  teleop_dashboard_node
  lut_calibrator_node
  sim_imu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Steering Lookup Table(LUT) CSV — 패키지 share에 설치해 이식성 있는 ament 경로로 로드
# (control_map_node가 get_package_share_directory("f1tenth_control")+"/cfg/..."로 조회)
install(FILES control_code/NUC6_glc_pacejka_lookup_table.csv
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_package()
