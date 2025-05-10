cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)


# 실행 파일
add_executable(${PROJECT_NAME} src/pure_pursuit.cpp)
# 의존성 연결
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  carla_msgs
  ackermann_msgs
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY track/
  DESTINATION share/${PROJECT_NAME}/track
)
ament_package()
