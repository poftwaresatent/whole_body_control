
set (CMAKE_VERBOSE_MAKEFILE ON)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init ()

add_definitions (-Wall -pipe)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set (LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library (wbc_opspace
  opspace/src/opspace.cpp
  opspace/src/Task.cpp
  opspace/src/controller_library.cpp
  )

rosbuild_add_executable (testTask opspace/src/testTask.cpp)
target_link_libraries (testTask wbc_opspace)
