cmake_minimum_required(VERSION 3.5)
project(map_builder)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(CGAL QUIET COMPONENTS)
find_package(Eigen3 REQUIRED)
find_package(PNG REQUIRED)
find_package(Boost REQUIRED)

if ( NOT CGAL_FOUND )
  message(WARNING "CGAL not found; CGAL-dependent targets will be skipped.\n"
                  "If you need reconstruction features, install CGAL and re-run CMake.")
  set(SKIP_RECON TRUE)
else()
  set(SKIP_RECON FALSE)
endif()

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MeshFromPointCloud2.srv"
  "srv/NormalsFromPointCloud2.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)

ament_export_include_directories(include)

# Only add the heavy reconstruction targets when CGAL (and related deps) are available.
if ( NOT SKIP_RECON )
  add_executable(map_builder_service
    src/recon_service_from_pointcloud.cpp
    src/hole_filling.cpp
    src/my_png.cpp
    src/pset.cpp
    src/rm_artifacts.cpp
    src/surface_recon.cpp
    src/texture.cpp
  )
  ament_target_dependencies(map_builder_service rclcpp sensor_msgs geometry_msgs visualization_msgs std_msgs pcl_conversions)

  add_executable(p_normal_service
    src/p_normal_service_from_pointcloud.cpp
    src/hole_filling.cpp
    src/my_png.cpp
    src/pset.cpp
    src/rm_artifacts.cpp
    src/surface_recon.cpp
    src/texture.cpp
  )
  ament_target_dependencies(p_normal_service rclcpp sensor_msgs geometry_msgs visualization_msgs std_msgs pcl_conversions)

  install(TARGETS
    map_builder_service
    p_normal_service
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
  )

  # Install python scripts as executables for ros2 run / launch
  install(PROGRAMS
    script/dyn_reconfigure_client_test.py
    script/dyn_reconfigure_server.py
    script/publish_mesh_marker.py
    script/test_mesh_service.py
    script/test_p_normal_service.py
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Skipping building C++ reconstruction nodes because CGAL was not found.")
endif()

# Always provide a minimal install rule so the package has an 'install' target
# (colcon invokes the install step even for packages that configure but don't
# build targets). Installing package.xml into the share folder is harmless
# and ensures the "install" target exists.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
