cmake_minimum_required(VERSION 3.22.1)

project(robot_slam
        LANGUAGES CXX C
        VERSION 0.0.1
        DESCRIPTION "robot slam source code"
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -DNDEBUG")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ROAMERX_DATA_ROOT "$ENV{ROAMERX_DATA_ROOT}" CACHE PATH
    "Runtime directory containing RoamerX maps and device data")
if(ROAMERX_DATA_ROOT STREQUAL "")
  set(ROAMERX_DATA_ROOT "/home/dogrobot/runtime/nx-edge/data/jszr")
endif()
add_definitions(-DROOT_DIR="${ROAMERX_DATA_ROOT}")
message(STATUS "RoamerX runtime data root: ${ROAMERX_DATA_ROOT}")

# Pick the OpenMP thread count from the actual core count. This used to be gated on
# CMAKE_SYSTEM_PROCESSOR matching x86, which silently left every aarch64 target
# (the Jetson Orin NX this runs on) at MP_PROC_NUM=1 with MP_EN undefined - the
# parallel path was dead code on the only hardware that ships.
# Override with -DROAMERX_MP_PROC_NUM=<n> to tune against CPU contention; 0 disables.
set(ROAMERX_MP_PROC_NUM "" CACHE STRING
    "OpenMP threads for the LIO front end; empty = auto from core count, 0 = disable")

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor number: ${N}")

if(NOT ROAMERX_MP_PROC_NUM STREQUAL "")
  set(_mp_proc_num ${ROAMERX_MP_PROC_NUM})
  message(STATUS "MP_PROC_NUM overridden by cache variable: ${_mp_proc_num}")
elseif(N GREATER 4)
  set(_mp_proc_num 3)
elseif(N GREATER 3)
  set(_mp_proc_num 2)
else()
  set(_mp_proc_num 1)
endif()

if(_mp_proc_num GREATER 1)
  add_definitions(-DMP_EN)
endif()
add_definitions(-DMP_PROC_NUM=${_mp_proc_num})
message(STATUS "core for MP: ${_mp_proc_num}")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(robots_dog_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  robots_dog_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenSSL REQUIRED)

file(GLOB_RECURSE SLAM_SDK
     CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER SLAM_SDK EXCLUDE REGEX ".*/enu_converter\\.cpp$")

include_directories(inlcude
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(mapping ${SLAM_SDK} test/mapping_demo.cpp)

add_executable(slam_enu_converter src/enu_converter.cpp)

add_executable(pcd2grid_streaming src/pcd2grid.cpp test/pcd2grid_streaming_cli.cpp)

target_link_libraries(mapping ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam OpenSSL::Crypto)
target_link_libraries(slam_enu_converter OpenSSL::Crypto)
target_link_libraries(pcd2grid_streaming ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

ament_target_dependencies(mapping ${dependencies})
ament_target_dependencies(slam_enu_converter rclcpp nav_msgs sensor_msgs)
ament_target_dependencies(pcd2grid_streaming ${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(global_factor_graph_test
    test/global_factor_graph_test.cpp
    src/global_factor_graph.cpp
  )
  if(TARGET global_factor_graph_test)
    target_include_directories(global_factor_graph_test PRIVATE
      ${EIGEN3_INCLUDE_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/include
    )
    target_link_libraries(global_factor_graph_test gtsam)
  endif()
  ament_add_gtest(enu_conversion_test test/enu_conversion_test.cpp)
  if(TARGET enu_conversion_test)
    target_include_directories(enu_conversion_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
  endif()
  ament_add_gtest(lidar_fov_filter_test
    test/lidar_fov_filter_test.cpp
    src/process/lidar_process.cpp
  )
  if(TARGET lidar_fov_filter_test)
    target_include_directories(lidar_fov_filter_test PRIVATE
      ${PCL_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/include
    )
    target_link_libraries(lidar_fov_filter_test ${PCL_LIBRARIES})
  endif()
  ament_add_gtest(ground_filter_test
    test/ground_filter_test.cpp
    src/process/ground_filter.cpp
  )
  if(TARGET ground_filter_test)
    target_include_directories(ground_filter_test PRIVATE
      ${PCL_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/include
    )
    target_link_libraries(ground_filter_test ${PCL_LIBRARIES})
  endif()
endif()

install(TARGETS mapping slam_enu_converter pcd2grid_streaming
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
