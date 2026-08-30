cmake_minimum_required(VERSION 3.5)
project(localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

cmake_policy(SET CMP0074 NEW)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(robots_dog_msgs REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

########################
## message generation ##
########################
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ScanMatchingStatus.msg
DEPENDENCIES
  std_msgs
  geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_lib SHARED
  src/localization/pose_estimator.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  nav_msgs
  ndt_omp
  fast_gicp
  robots_dog_msgs
)
rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_node
  apps/localization_nodelet.cpp
  src/localization/pose_estimator.cpp
  src/localization/static_imu_init.cpp
  src/localization/global_localization.cpp
  src/localization/scan_context_db.cpp
  src/localization/relocalization_geometry.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  sensor_msgs
  geometry_msgs
  nav_msgs
  ndt_omp
  fast_gicp
  robots_dog_msgs
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)

# Offline scoring for the scan context retrieval. Links only PCL and Eigen, so it runs
# against recorded maps without a ROS graph, a map service or the robot.
add_executable(${PROJECT_NAME}_scan_context_check
  tools/scan_context_check.cpp
  src/localization/scan_context_db.cpp
)

# Shadow/offline Tier-1 validation: Scan Context proposals must survive real 3D
# FastGICP, bidirectional overlap, Hessian and an independent ICP cross-check.
add_executable(${PROJECT_NAME}_relocalization_geometry_check
  tools/relocalization_geometry_check.cpp
  src/localization/scan_context_db.cpp
  src/localization/relocalization_geometry.cpp
)
ament_target_dependencies(${PROJECT_NAME}_relocalization_geometry_check
  pcl_ros
  pcl_conversions
  fast_gicp
)
target_include_directories(${PROJECT_NAME}_relocalization_geometry_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_scan_context_check
  pcl_ros
  pcl_conversions
)
target_include_directories(${PROJECT_NAME}_scan_context_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_scan_context_check
    ${PROJECT_NAME}_relocalization_geometry_check
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
        FILES ../run_localization.sh
        DESTINATION .
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(point_cloud_scheduler_test
    test/point_cloud_scheduler_test.cpp
  )
  target_include_directories(point_cloud_scheduler_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(lio_motion_guard_test
    test/lio_motion_guard_test.cpp
  )
  target_include_directories(lio_motion_guard_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(scan_match_policy_test
    test/scan_match_policy_test.cpp
  )
  target_include_directories(scan_match_policy_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(global_relocalization_policy_test
    test/global_relocalization_policy_test.cpp
  )
  target_include_directories(global_relocalization_policy_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(correction_policy_test
    test/correction_policy_test.cpp
  )
  target_include_directories(correction_policy_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(relocalization_geometry_policy_test
    test/relocalization_geometry_policy_test.cpp
    src/localization/relocalization_geometry.cpp
  )
  ament_target_dependencies(relocalization_geometry_policy_test
    pcl_ros
    pcl_conversions
    fast_gicp
  )
  target_include_directories(relocalization_geometry_policy_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
