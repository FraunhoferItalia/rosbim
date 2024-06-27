cmake_minimum_required(VERSION 3.8)

project(rosbim_export_geometry_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bim_interface REQUIRED)
find_package(rosbim_example_models REQUIRED)
find_package(rosbim_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


set(srv_files
  "srv/ExportGeometryService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  rosbim_export_geometry_plugin/export_geometry_plugin.py
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
