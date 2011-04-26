cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Release)

add_definitions (-Wall -pipe)
rosbuild_init ()

set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library (wbc_uta_opspace
  uta_opspace/strutil.cpp
  uta_opspace/ControllerNG.cpp
  uta_opspace/HelloGoodbyeSkill.cpp
  uta_opspace/DelayHistogram.cpp
  )
