cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (ROS_BUILD_TYPE Release)

rosbuild_init ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library (wbc_urdf
  src/Model.cpp
  src/urdf_dump.cpp
  src/urdf_to_tao.cpp
  )

add_executable (urdf_to_tao src/urdf_to_tao_node.cpp)
target_link_libraries (urdf_to_tao wbc_urdf)
