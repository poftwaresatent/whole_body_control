cmake_minimum_required (VERSION 2.4.6)

set (CMAKE_VERBOSE_MAKEFILE ON)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions (-Wall -pipe)
rosbuild_init ()

rosbuild_genmsg ()
rosbuild_gensrv ()

rosbuild_add_library (wbc_pr2_ctrl
  src/mq_robot_api.cpp
  src/opspace_planar_controller.cpp
  )
target_link_libraries (wbc_pr2_ctrl wbc_core wbc_urdf)

rosbuild_add_executable (bin/jspace_servo src/jspace_servo.cpp)
target_link_libraries (bin/jspace_servo wbc_pr2_ctrl)

rosbuild_add_executable (bin/dynamic_model src/dynamic_model_node.cpp)
target_link_libraries (bin/dynamic_model wbc_pr2_ctrl)

rosbuild_add_executable (bin/dynamic_model_dump src/dynamic_model_dump_node.cpp)
target_link_libraries (bin/dynamic_model_dump wbc_pr2_ctrl)

rosbuild_add_library (wbc_pr2_plugins
  src/pump_plugin.cpp
  src/wbc_plugin.cpp
  )

target_link_libraries (wbc_pr2_plugins wbc_pr2_ctrl)
