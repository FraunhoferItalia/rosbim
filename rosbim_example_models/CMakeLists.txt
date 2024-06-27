cmake_minimum_required(VERSION 3.8)
project(rosbim_example_models)

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})


ament_package()
