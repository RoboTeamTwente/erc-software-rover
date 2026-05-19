cmake_minimum_required(VERSION 3.8)
project(rtt_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the packages (Added ament_index_cpp here)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 2. Create the executable
add_executable(bt_node src/ex.cpp src/main.cpp)

# 3. Handle Dependencies (The clean way)
# This handles the include directories AND the linking for you
ament_target_dependencies(bt_node
  behaviortree_cpp
  ament_index_cpp
)

# 4. Install
install(TARGETS bt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()