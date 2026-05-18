cmake_minimum_required(VERSION 3.8)
project(robot_navigation_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS
  navigation_node
  DESTINATION lib/
)

ament_package()

add_executable(sensor_test_node src/sensor_test_node.cpp)
ament_target_dependencies(sensor_test_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  sensor_test_node
  DESTINATION lib/
)
