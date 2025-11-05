cmake_minimum_required(VERSION 3.22)
project(map_builder VERSION 1.0.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtabmap_slam REQUIRED)
find_package(rtabmap_viz REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_util REQUIRED)
find_package(rtabmap_msgs REQUIRED)

# Install launch files and config
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()