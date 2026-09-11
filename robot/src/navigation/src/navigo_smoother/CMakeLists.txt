cmake_minimum_required(VERSION 3.8)
project(navigo_smoother)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

add_library(navigo_passthrough_smoother SHARED
  src/passthrough_smoother.cpp)
target_include_directories(navigo_passthrough_smoother PUBLIC include)
ament_target_dependencies(navigo_passthrough_smoother
  nav2_core nav_msgs pluginlib rclcpp rclcpp_lifecycle tf2_ros nav2_costmap_2d)
pluginlib_export_plugin_description_file(nav2_core navigo_smoother_plugins.xml)

install(TARGETS navigo_passthrough_smoother
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES navigo_smoother_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_package()
