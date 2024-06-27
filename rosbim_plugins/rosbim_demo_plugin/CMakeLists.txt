cmake_minimum_required(VERSION 3.8)
project(rosbim_demo_plugin)

find_package(ament_cmake REQUIRED)
find_package(bim_interface REQUIRED)
find_package(rosbim_example_models REQUIRED)
find_package(rosbim_manager_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)


install(PROGRAMS
  src/rosbim_demo_plugin/demo_plugin.py
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


ament_export_dependencies(rosidl_default_runtime)

ament_package()
