cmake_minimum_required(VERSION 3.8)
project(navigo_smoother)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(navigo_core REQUIRED)
find_package(navigo_costmap_2d REQUIRED)
find_package(navigo_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

set(dependencies
  angles nav2_msgs nav_2d_utils navigo_core navigo_costmap_2d navigo_util
  nav_msgs pluginlib rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle
  tf2 tf2_geometry_msgs tf2_ros)

add_library(smoother_server_core SHARED src/navigo_smoother.cpp)
target_include_directories(smoother_server_core PUBLIC include)
ament_target_dependencies(smoother_server_core ${dependencies})

add_executable(smoother_server src/main.cpp)
target_link_libraries(smoother_server smoother_server_core)
ament_target_dependencies(smoother_server ${dependencies})

add_library(navigo_simple_smoother SHARED src/simple_smoother.cpp)
target_include_directories(navigo_simple_smoother PUBLIC include)
ament_target_dependencies(navigo_simple_smoother ${dependencies})

add_library(navigo_passthrough_smoother SHARED src/passthrough_smoother.cpp)
target_include_directories(navigo_passthrough_smoother PUBLIC include)
ament_target_dependencies(navigo_passthrough_smoother ${dependencies})

pluginlib_export_plugin_description_file(navigo_core navigo_smoother_plugins.xml)
rclcpp_components_register_nodes(smoother_server_core "navigo_smoother::SmootherServer")

install(TARGETS smoother_server_core navigo_simple_smoother navigo_passthrough_smoother
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS smoother_server RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES navigo_smoother_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(smoother_server_core navigo_simple_smoother navigo_passthrough_smoother)
ament_export_dependencies(${dependencies})
ament_package()
