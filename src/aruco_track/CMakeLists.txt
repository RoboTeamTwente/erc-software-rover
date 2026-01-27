cmake_minimum_required(VERSION 3.20)
project(aruco_track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(aruco_tf2_node DIRECTORY src)

# OpenCV is ✨Special✨
target_include_directories(aruco_tf2_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_tf2_node ${OpenCV_LIBRARIES})

install(
  FILES config/charuco_camera_params.yml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS aruco_tf2_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
