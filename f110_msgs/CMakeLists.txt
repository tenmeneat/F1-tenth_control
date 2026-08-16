cmake_minimum_required(VERSION 3.8)
project(f110_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/CarState.msg"
  "msg/CarStateStamped.msg"
  "msg/GapData.msg"
  "msg/LapData.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/OTWpntArray.msg"
  "msg/OpponentTrajectory.msg"
  "msg/OpponentTrajectories.msg"
  "msg/OppWpnt.msg"
  "msg/PidData.msg"
  "msg/ProjOppPoint.msg"
  "msg/ProjOppTraj.msg"
  "msg/StateMachine.msg"
  "msg/Wpnt.msg"
  "msg/WpntArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
