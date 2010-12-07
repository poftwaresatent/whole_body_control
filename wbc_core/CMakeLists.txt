cmake_minimum_required (VERSION 2.4.6)

set (CMAKE_VERBOSE_MAKEFILE ON)
#set (ROS_BUILD_TYPE Release)
set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions (-Wall -pipe)
rosbuild_init ()

#rosbuild_genmsg ()
#rosbuild_gensrv ()

include_directories (
  src/jspace
  src/tao
  src/wbc_tinyxml
  )

rosbuild_add_library (wbc_core
  # WBC's in-source snapshot of TinyXML
  src/wbc_tinyxml/wbc_tinyxml/wbc_tinystr.cpp
  src/wbc_tinyxml/wbc_tinyxml/wbc_tinyxml.cpp
  src/wbc_tinyxml/wbc_tinyxml/wbc_tinyxmlerror.cpp
  src/wbc_tinyxml/wbc_tinyxml/wbc_tinyxmlparser.cpp
  
  # The TAO dynamics engine for rigid-body branching structures
  src/tao/tao/dynamics/taoABDynamics.cpp
  src/tao/tao/dynamics/taoABJoint.cpp
  src/tao/tao/dynamics/taoABNode.cpp
  src/tao/tao/dynamics/taoCNode.cpp
  src/tao/tao/dynamics/taoDynamics.cpp
  src/tao/tao/dynamics/taoGroup.cpp
  src/tao/tao/dynamics/taoJoint.cpp
  src/tao/tao/dynamics/taoNode.cpp
  src/tao/tao/dynamics/taoWorld.cpp
  src/tao/tao/matrix/TaoDeMatrix3f.cpp
  src/tao/tao/matrix/TaoDeMatrix6.cpp
  src/tao/tao/matrix/TaoDeQuaternionf.cpp
  src/tao/tao/matrix/TaoDeTransform.cpp
  src/tao/tao/matrix/TaoDeVector6.cpp
  src/tao/tao/utility/TaoDeLogger.cpp
  src/tao/tao/utility/TaoDeMassProp.cpp
  
  # The core joint-space dynamic model (a facade for TAO with some further goodies)
  src/jspace/jspace/Model.cpp
  src/jspace/jspace/State.cpp
  src/jspace/jspace/Status.cpp
  src/jspace/jspace/Controller.cpp
  src/jspace/jspace/controller_library.cpp
  src/jspace/jspace/tao_dump.cpp
  src/jspace/jspace/tao_util.cpp
  src/jspace/jspace/vector_util.cpp
  src/jspace/jspace/wrap_eigen.cpp
  src/jspace/jspace/inertia_util.cpp
  src/jspace/jspace/test/sai_brep.cpp
  src/jspace/jspace/test/sai_brep_parser.cpp
  )

#target_link_libraries (wbc_core
#  )

#rosbuild_add_executable (bin/jspace_servo src/jspace_servo.cpp)
#target_link_libraries (bin/jspace_servo wbc_core)
