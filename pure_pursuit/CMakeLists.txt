cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(carla_msgs REQUIRED)

include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  include/controller.h
  include/pure_pursuit.h
  include/ros_data.h

  src/controller.cpp
  src/pure_pursuit.cpp
  src/ros_data.cpp

)
ament_target_dependencies(${PROJECT_NAME}
ackermann_msgs
geometry_msgs
nav_msgs
rclcpp
std_msgs
tf2
tf2_ros
visualization_msgs
carla_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
