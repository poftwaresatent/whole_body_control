cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Debug)
add_definitions (-Wall -pipe)
rosbuild_init ()
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_library (wbc_m3_ctrl
  src/foo.cpp
  )
target_link_libraries (wbc_m3_ctrl wbc_opspace)

rosbuild_add_executable (bar src/bar.cpp)
target_link_libraries (bar wbc_m3_ctrl)
