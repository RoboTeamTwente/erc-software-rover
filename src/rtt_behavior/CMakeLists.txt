cmake_minimum_required(VERSION 3.8)
project(rtt_behaviour)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 1. Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "actions/CollectTask.action"
  "actions/HardwareCommand.action"
  DEPENDENCIES unique_identifier_msgs action_msgs
)

# 2. Create the executable
add_executable(bt_node src/ex.cpp src/main.cpp)

# 3. Handle External Dependencies
ament_target_dependencies(bt_node
  behaviortree_cpp
  ament_index_cpp
  behaviortree_ros2
  std_msgs
  rclcpp
  turtlesim
)

# 4. CRITICAL: Link your custom actions to the executable
# This allows #include "rtt_behaviour/action/collect_task.hpp" to work
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(bt_node "${cpp_typesupport_target}")

# 5. Install
install(TARGETS bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()