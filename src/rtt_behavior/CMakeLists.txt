cmake_minimum_required(VERSION 3.8)
project(rtt_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(bt_node DIRECTORY src)

# 1. Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CollectTask.action"
  "action/HardwareCommand.action"
  DEPENDENCIES unique_identifier_msgs action_msgs
)

# 4. CRITICAL: Link your custom actions to the executable
# This allows #include "rtt_behavior/action/collect_task.hpp" to work
rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(bt_node "${CPP_TYPESUPPORT_TARGET}")

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_auto_package()
