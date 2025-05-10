cmake_minimum_required(VERSION 3.5)
project(mobile_system_control)

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# include 디렉토리 설정
include_directories(
  include
)

# 실행 파일 추가
add_executable(mobile_system_control 
  src/main.cpp
  src/mobile_system_control.cpp
)

# ROS2 패키지와 연결
ament_target_dependencies(mobile_system_control 
  rclcpp
  carla_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

# 설치 설정
install(TARGETS
  mobile_system_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
# 패키지 등록
ament_package()
