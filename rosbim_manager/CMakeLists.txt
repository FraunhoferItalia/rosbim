cmake_minimum_required(VERSION 3.8)
project(rosbim_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosbim_manager_msgs REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  rosbim_manager/rosbim_manager.py
  rosbim_manager/spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
