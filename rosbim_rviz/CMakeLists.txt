cmake_minimum_required(VERSION 3.5)
project(rosbim_rviz)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rosbim_export_geometry_plugin REQUIRED)


include_directories(
  include
)


set(HEADER_FILES
  include/mesh_selection_handler.hpp
  include/mesh.hpp
  include/rosbim_display.hpp
  include/rosbim_visual.hpp
  include/properties/ros_service_property.hpp
)

set(SRC_FILES
  src/mesh_selection_handler.cpp
  src/mesh.cpp
  src/rosbim_display.cpp
  src/rosbim_visual.cpp
  src/properties/ros_service_property.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HEADER_FILES})

set(dependencies
  Qt5
  rclcpp
  rclcpp_lifecycle
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  rviz_default_plugins
  rosbim_export_geometry_plugin
  pluginlib
)

ament_target_dependencies(${PROJECT_NAME} 
  ${dependencies}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")



pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${LIBRARY_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Qt5
  rviz_common
  rclcpp
)

ament_package()