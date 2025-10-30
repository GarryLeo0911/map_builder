cmake_minimum_required(VERSION 3.8)
project(map_builder)

# Suppress CMake policy warnings from PCL
cmake_policy(SET CMP0074 NEW)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features surface keypoints registration)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Define PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Point Cloud Processor Node
add_executable(point_cloud_processor 
  src/point_cloud_processor_node.cpp
  src/point_cloud_processor.cpp
)

ament_target_dependencies(point_cloud_processor
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
)

target_link_libraries(point_cloud_processor
  ${PCL_LIBRARIES}
)

# Map Builder Node
add_executable(map_builder_node 
  src/map_builder_node.cpp
  src/map_builder.cpp
)

ament_target_dependencies(map_builder_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
)

target_link_libraries(map_builder_node
  ${PCL_LIBRARIES}
)

# Surface Reconstructor Node
add_executable(surface_reconstructor 
  src/surface_reconstructor_node.cpp
  src/surface_reconstructor.cpp
)

ament_target_dependencies(surface_reconstructor
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(surface_reconstructor
  ${PCL_LIBRARIES}
)

# Enhanced Visual Odometry Node (with IMU fusion and feature tracking)
add_executable(enhanced_visual_odometry_node 
  src/enhanced_visual_odometry_node.cpp
  src/enhanced_visual_odometry.cpp
)

ament_target_dependencies(enhanced_visual_odometry_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
  cv_bridge
)

target_link_libraries(enhanced_visual_odometry_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Visual Odometry Node (original)
add_executable(visual_odometry 
  src/visual_odometry_node.cpp
  src/visual_odometry.cpp
)

ament_target_dependencies(visual_odometry
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
)

target_link_libraries(visual_odometry
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  point_cloud_processor
  map_builder_node
  surface_reconstructor
  visual_odometry
  enhanced_visual_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install resource files
install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_map_builder.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PCL_LIBRARIES})
    ament_target_dependencies(${PROJECT_NAME}_test
      rclcpp
      sensor_msgs
      pcl_ros
      pcl_conversions
    )
  endif()
endif()

ament_package()