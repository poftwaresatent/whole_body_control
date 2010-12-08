cmake_minimum_required (VERSION 2.4.6)

set (CMAKE_VERBOSE_MAKEFILE ON)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions (-Wall)
rosbuild_init ()

rosbuild_genmsg ()
rosbuild_gensrv ()

## jspace uses Eigen instead of SAIMAtrix, need to adapt opspace_wristpos_controller
## rosbuild_add_library (pr2_stanford_wbc src/mq_robot_api.cpp src/opspace_wristpos_controller.cpp)
rosbuild_add_library (pr2_stanford_wbc src/mq_robot_api.cpp src/opspace_planar_controller.cpp src/DelayHistogram.cpp)
target_link_libraries (pr2_stanford_wbc Stanford_WBC lapack)

rosbuild_add_executable (bin/jspace_servo src/jspace_servo.cpp)
target_link_libraries (bin/jspace_servo pr2_stanford_wbc)

rosbuild_add_executable (bin/dynamic_model src/dynamic_model_node.cpp)
target_link_libraries (bin/dynamic_model pr2_stanford_wbc)

rosbuild_add_executable (bin/dynamic_model_dump src/dynamic_model_dump_node.cpp)
target_link_libraries (bin/dynamic_model_dump pr2_stanford_wbc)

rosbuild_add_executable (bin/dump_tao src/dump_tao_node.cpp)
target_link_libraries (bin/dump_tao pr2_stanford_wbc)

rosbuild_add_executable (bin/feed_playtao src/feed_playtao_node.cpp)
target_link_libraries (bin/feed_playtao pr2_stanford_wbc)

rosbuild_add_executable (bin/testMQPump src/testMQPump.cpp)
target_link_libraries (bin/testMQPump pr2_stanford_wbc)

rosbuild_add_executable (bin/testMQServo src/testMQServo.cpp)
target_link_libraries (bin/testMQServo pr2_stanford_wbc)

rosbuild_add_executable (bin/testXmlRpcParams src/testXmlRpcParams.cpp)
target_link_libraries (bin/testXmlRpcParams pr2_stanford_wbc)

rosbuild_add_library (pump src/pump.cpp)
target_link_libraries (pump pr2_stanford_wbc)
