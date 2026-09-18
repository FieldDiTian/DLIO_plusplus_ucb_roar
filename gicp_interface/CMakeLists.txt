###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.16)
project(gicp_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(race_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(LibXml2 REQUIRED)  # URDF parsing for offline aux-LiDAR extrinsics (lidar_concat)
# small_gicp is supplied by the race_common workspace.  Keep this as an
# installed CMake dependency rather than vendoring another checkout here;
# callers can point CMake at it with -Dsmall_gicp_DIR=... or
# CMAKE_PREFIX_PATH=....
find_package(small_gicp CONFIG REQUIRED)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

# Pure registration-quality math is built separately so it can be unit-tested
# without constructing a ROS node.
add_library(gicp_interface_quality STATIC src/quality_gate.cpp)
target_include_directories(gicp_interface_quality PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gicp_interface_quality PUBLIC Eigen3::Eigen)

# Static race_common TTL parsing and candidate validation, kept independent of
# ROS so its geometry and policy can be regression-tested in isolation.
add_library(gicp_interface_track_constraint STATIC src/ttl_track_constraint.cpp)
target_include_directories(gicp_interface_track_constraint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gicp_interface_track_constraint PUBLIC Eigen3::Eigen)

# Timestamp-consistent inertial state used by the production node.
set(GICP_FILTER_CORE_SOURCES
  src/delayed_gicp_fusion.cpp
  src/impl/delayed_ieskf.cpp
  src/impl/imu_trajectory.cpp
  src/impl/planar_ieskf.cpp
  src/impl/planar_measurement.cpp
  src/impl/planar_pose.cpp
  src/impl/spatial_measurement.cpp
  src/impl/voxel_surfel_map.cpp
)
add_library(gicp_interface_filter_core STATIC ${GICP_FILTER_CORE_SOURCES})
target_include_directories(gicp_interface_filter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gicp_interface_filter_core PUBLIC
  Eigen3::Eigen
)

# Keep the source layout parallel to perception-ws: a thin executable plus
# focused implementation translation units under src/impl.
# The node body lives in a component library so it can share one process -- and
# therefore intra-process point clouds -- with the Iris driver.
add_library(gicp_interface_component SHARED
  src/localizer_utils.cpp
  src/impl/node.cpp
  src/impl/parameters.cpp
  src/impl/map.cpp
  src/impl/local_map.cpp
  src/impl/initialization.cpp
  src/impl/lidar_sync.cpp
  src/impl/preprocessing.cpp
  src/impl/registration.cpp
  src/impl/outputs.cpp
  src/impl/recovery.cpp
  src/impl/prediction.cpp
  src/impl/fusion.cpp
)
target_include_directories(gicp_interface_component PRIVATE ${LIBXML2_INCLUDE_DIR})
target_link_libraries(gicp_interface_component
  ${PCL_LIBRARIES}
  Threads::Threads
  OpenMP::OpenMP_CXX
  small_gicp::small_gicp
  gicp_interface_quality
  gicp_interface_track_constraint
  gicp_interface_filter_core
  ${LIBXML2_LIBRARIES}
)
ament_target_dependencies(gicp_interface_component "tf2_ros" "tf2_geometry_msgs" "tf2_eigen" "tf2_sensor_msgs" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "race_msgs" "sensor_msgs" "geometry_msgs")

rclcpp_components_register_nodes(gicp_interface_component
  "gicp_localizer::GicpLocalizer")

# Standalone entry point, kept so the localizer can still run in its own process.
add_executable(gicp_interface_node src/gicp_interface_node.cpp)
target_link_libraries(gicp_interface_node gicp_interface_component)
ament_target_dependencies(gicp_interface_node "rclcpp")

# Linked into the shared component library above, so all three need PIC.
set_target_properties(
  gicp_interface_quality
  gicp_interface_track_constraint
  gicp_interface_filter_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp race_msgs sensor_msgs geometry_msgs pcl_ros small_gicp)

# Binaries and the small runtime configuration surface.  Replay QoS, RViz,
# and optional TTL profiles remain local-only and are not installed into the
# deployable package.
install(TARGETS gicp_interface_node
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS gicp_interface_component
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)
install(FILES
  param/gicp_interface.param.yaml
  param/laguna.param.yaml
  DESTINATION share/${PROJECT_NAME}/param)
install(FILES
  param/laguna.param.yaml
  DESTINATION share/${PROJECT_NAME}/param
  RENAME race_common_laguna_baseline.param.yaml)
install(PROGRAMS launch/localize.launch.py
        DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
