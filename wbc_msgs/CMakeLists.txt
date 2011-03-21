cmake_minimum_required (VERSION 2.4.6)

#set (CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Release)
#set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init ()
rosbuild_genmsg ()
rosbuild_gensrv ()
