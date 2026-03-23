cmake_minimum_required(VERSION 3.20)
project(rtt_rover_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)
FetchContent_Declare(
  erc_control_steering
  GIT_REPOSITORY ssh://git@github.com/RoboTeamTwente/erc-control-steering
  GIT_TAG 4cb1d60f13cdb50bc48b3fecc6efb08686de73ee)

FetchContent_MakeAvailable(erc_control_steering)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(rtt_rover_control SHARED
  ${erc_control_steering_SOURCE_DIR}/control_ert_rtw/control.c)

install(FILES
  ${erc_control_steering_SOURCE_DIR}/control_ert_rtw/control.h
  ${erc_control_steering_SOURCE_DIR}/control_ert_rtw/rtwtypes.h
  DESTINATION include/rtt_rover_control)

ament_auto_package()
