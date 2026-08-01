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
# (2026-08-01: MPPI GPU 솔버 제거로 CUDA(.cu) 빌드 유닛이 없어져 CXX 언어 스코프 제한이
#  더 이상 필수는 아니지만, 순수 C++ 패키지로 남는 한 부작용이 없어 그대로 둔다.)
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
# 2. Control MAP 노드 빌드 (L1 Guidance + Steering LUT)
# ==============================================================================
add_executable(control_map_node
  control_code/control_map_node.cpp
  control_code/gap_follower.cpp
)
# ⚠️ StabilityController는 2026-07-29 롤 인지 ESC 제거와 함께 **헤더 전용**(.hpp)이 됐다 —
#    imu_stability_controller.cpp는 삭제됐으므로 여기 다시 추가하지 말 것.
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

# ==============================================================================
# 6. Drive Source Selector 노드 (자율 명령 포워더 — sim/real 공용)
#    수동/자율/E-stop Mux(teleop)는 이 저장소 밖 담당이다: 실차는 f1tenth_stack
#    (drive_mode_manager+ackermann_mux), 시뮬은 Mux 없이 이 노드가 자율 명령을 /drive로
#    직결한다. control_map_node의 /drive_autonomous를 그대로 /drive로 포워딩한다.
#    (2026-08-01: MPPI 노드/솔버 전체 제거와 함께 MAP/MPPI 셀렉터 기능도 걷어냄 — 대회
#    준비 기간 동안 MAP 하나에만 집중하기로 함. MPPI 코드는 git 이력에서 꺼낼 수 있음.)
# ==============================================================================
add_executable(drive_source_selector control_code/drive_source_selector.cpp)
target_link_libraries(drive_source_selector
  rclcpp::rclcpp
  ${ackermann_msgs_TARGETS}
)

# ==============================================================================
# 6-B. Odom 거리 스케일 보정 노드 (실차 원격 — 우리 컴에서 실행)
#    "명령 주고 자로 재기" 테스트 자동화. odom 한 토픽 안의 twist(VESC 휠 패스스루)와
#    pose(MCL 스캔매칭)를 따로 적분해 명령/휠/맵 세 거리를 동시 측정 → 어느 게인이
#    틀렸는지 분리해준다. /drive 미발행(순수 관찰자).
# ==============================================================================
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

# Steering Lookup Table(LUT) CSV — 패키지 share에 설치해 이식성 있는 ament 경로로 로드
# (control_map_node가 get_package_share_directory("f1tenth_control")+"/cfg/..."로 조회)
install(FILES control_code/NUC6_glc_pacejka_lookup_table.csv
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_package()
