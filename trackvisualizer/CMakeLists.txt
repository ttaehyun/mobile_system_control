cmake_minimum_required(VERSION 3.5)
project(trackvisualizer)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# include 디렉토리 설정
include_directories(
  include
)

add_executable(trackvisualizer 
  src/main.cpp
  src/trackvisualizer.cpp
)

# ROS2 패키지와 연결
ament_target_dependencies(trackvisualizer 
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)


install(TARGETS
  trackvisualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY 
    launch/
    track/
    rviz/
  DESTINATION 
    share/${PROJECT_NAME}
)
# 패키지 등록
ament_package()