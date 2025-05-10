cmake_minimum_required(VERSION 3.5)
project(pid_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(carla_msgs REQUIRED)
# Include directories
include_directories(
  include
)

# Build the node
add_executable(PID_control_ex_node
  src/pid_controller.cpp
  src/main.cpp
)
ament_target_dependencies(PID_control_ex_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  carla_msgs
)

install(TARGETS 
  PID_control_ex_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    track/
    launch/
  DESTINATION
    share/${PROJECT_NAME}
)
ament_package()
