cmake_minimum_required(VERSION 3.8)
project(rosbim_export_map_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bim_interface REQUIRED)
find_package(rosbim_example_models REQUIRED)
find_package(rosbim_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

set(srv_files
  "srv/ExportMapService.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

install(PROGRAMS
  rosbim_export_map_plugin/export_map_plugin.py
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rosidl_default_runtime)

ament_package()
