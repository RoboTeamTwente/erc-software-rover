
cmake_minimum_required(VERSION 3.20)
project(comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS / ament ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- Protobuf ---
find_package(Protobuf REQUIRED)

include(FetchContent)

FetchContent_Declare(
  erc_protobufs
  GIT_REPOSITORY https://github.com/RoboTeamTwente/ERC-Protobufs.git
  GIT_TAG c0e423bbd5cf930268cc5e69a8399b4d0ba1412d
)
FetchContent_MakeAvailable(erc_protobufs)

# Where generated .pb.h/.pb.cc will go
set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${PROTO_BINARY_DIR}")

# Collect every .proto from the fetched repo
# Collect ONLY component protos (avoid top-level *_board.proto that duplicate symbols)
file(GLOB_RECURSE ERC_PROTO_FILES
  "${erc_protobufs_SOURCE_DIR}/components/*.proto"
)

# If you need root-level common.proto, include it explicitly
list(APPEND ERC_PROTO_FILES
  "${erc_protobufs_SOURCE_DIR}/common.proto"
)

# ---- Generate explicitly (preserve folder structure) ----
set(ERC_PROTO_SRCS "")
set(ERC_PROTO_HDRS "")

foreach(f ${ERC_PROTO_FILES})
  file(RELATIVE_PATH rel "${erc_protobufs_SOURCE_DIR}" "${f}")
  get_filename_component(rel_dir "${rel}" DIRECTORY)
  get_filename_component(base "${rel}" NAME_WE)

  file(MAKE_DIRECTORY "${PROTO_BINARY_DIR}/${rel_dir}")

  list(APPEND ERC_PROTO_SRCS "${PROTO_BINARY_DIR}/${rel_dir}/${base}.pb.cc")
  list(APPEND ERC_PROTO_HDRS "${PROTO_BINARY_DIR}/${rel_dir}/${base}.pb.h")
endforeach()

add_custom_command(
  OUTPUT ${ERC_PROTO_SRCS} ${ERC_PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS
    --cpp_out="${PROTO_BINARY_DIR}"
    -I "${erc_protobufs_SOURCE_DIR}"
    ${ERC_PROTO_FILES}
  DEPENDS ${ERC_PROTO_FILES}
  COMMENT "Generating C++ protobuf sources"
  VERBATIM
)

# Compile generated .pb.cc into an object library
add_library(erc_proto_objects OBJECT
  ${ERC_PROTO_SRCS}
)

target_include_directories(erc_proto_objects PUBLIC
  $<BUILD_INTERFACE:${PROTO_BINARY_DIR}>
)

target_link_libraries(erc_proto_objects PUBLIC
  protobuf::libprotobuf
)

# --- Your ROS node executable ---
ament_auto_add_executable(comms_node
  src/udp_forwarder_node.cpp
)

# Link generated protos into the node (plain signature)
target_link_libraries(comms_node
  erc_proto_objects
  protobuf::libprotobuf
)

# target_include_directories(comms_node PRIVATE
#   ${PROTO_BINARY_DIR}
# )

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
