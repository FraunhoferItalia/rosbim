cmake_minimum_required(VERSION 3.8)

project(rosbim_manager_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Plugin.msg"
  "msg/PluginState.msg"
)

set(srv_files
  "srv/ListPlugins.srv"
  "srv/LoadBimFile.srv"
  "srv/LoadPlugin.srv"
  "srv/SpawnPlugin.srv"
  "srv/StartPlugin.srv"
  "srv/StopPlugin.srv"
  "srv/SwitchBackend.srv"
  "srv/UnloadPlugin.srv"
)

# set(action_files
# )


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  #${action_files}
  DEPENDENCIES std_msgs std_srvs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
