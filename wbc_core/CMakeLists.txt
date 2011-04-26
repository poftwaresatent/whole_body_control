cmake_minimum_required (VERSION 2.4.6)

set (CMAKE_VERBOSE_MAKEFILE ON)
set (ROS_BUILD_TYPE Release)
#set (ROS_BUILD_TYPE Debug)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions (-Wall -pipe)
rosbuild_init ()

# The wbc_tinyxml includes can be quite problematic in case the
# compiler first finds a system-installed or ROS-3rdparty TinyXML
# header. This causes headaches especially when when some other
# package depends on us but the TinyXML that was found instead of ours
# was compiled with different flags. The propoer solution would be to
# completely hide our TinyXML snapshot, it is only used for the legacy
# SAI XML parser anyway and could thus be stashed into a
# self-contained subdirectory.
add_definitions (-DTIXML_USE_STL)

include_directories (
  stanford_wbc/jspace
  stanford_wbc/tao
  stanford_wbc/opspace/include
  stanford_wbc/3rdparty/wbc_tinyxml
  stanford_wbc/3rdparty/reflexxes_otg/include
  )

rosbuild_add_library (wbc_core

  stanford_wbc/3rdparty/reflexxes_otg/src/TypeIOTGProfiles.cpp
  stanford_wbc/3rdparty/reflexxes_otg/src/TypeIOTGMath.cpp
  stanford_wbc/3rdparty/reflexxes_otg/src/TypeIOTGPolynomial.cpp
  stanford_wbc/3rdparty/reflexxes_otg/src/TypeIOTG.cpp
  stanford_wbc/3rdparty/reflexxes_otg/src/TypeIOTGDecision.cpp
  
  stanford_wbc/3rdparty/wbc_tinyxml/wbc_tinyxml/wbc_tinyxmlerror.cpp
  stanford_wbc/3rdparty/wbc_tinyxml/wbc_tinyxml/wbc_tinyxmlparser.cpp
  stanford_wbc/3rdparty/wbc_tinyxml/wbc_tinyxml/wbc_tinyxml.cpp
  stanford_wbc/3rdparty/wbc_tinyxml/wbc_tinyxml/wbc_tinystr.cpp
  
  stanford_wbc/tao/tao/dynamics/taoCNode.cpp
  stanford_wbc/tao/tao/dynamics/taoJoint.cpp
  stanford_wbc/tao/tao/dynamics/taoABJoint.cpp
  stanford_wbc/tao/tao/dynamics/taoABDynamics.cpp
  stanford_wbc/tao/tao/dynamics/taoABNode.cpp
  stanford_wbc/tao/tao/dynamics/taoGroup.cpp
  stanford_wbc/tao/tao/dynamics/taoNode.cpp
  stanford_wbc/tao/tao/dynamics/taoDynamics.cpp
  stanford_wbc/tao/tao/dynamics/taoWorld.cpp
  stanford_wbc/tao/tao/matrix/TaoDeQuaternionf.cpp
  stanford_wbc/tao/tao/matrix/TaoDeTransform.cpp
  stanford_wbc/tao/tao/matrix/TaoDeMatrix3f.cpp
  stanford_wbc/tao/tao/matrix/TaoDeMatrix6.cpp
  stanford_wbc/tao/tao/matrix/TaoDeVector6.cpp
  stanford_wbc/tao/tao/utility/TaoDeLogger.cpp
  stanford_wbc/tao/tao/utility/TaoDeMassProp.cpp
  
  stanford_wbc/jspace/jspace/Status.cpp
  stanford_wbc/jspace/jspace/controller_library.cpp
  stanford_wbc/jspace/jspace/inertia_util.cpp
  stanford_wbc/jspace/jspace/State.cpp
  stanford_wbc/jspace/jspace/wrap_eigen.cpp
  stanford_wbc/jspace/jspace/vector_util.cpp
  stanford_wbc/jspace/jspace/strutil.cpp
  stanford_wbc/jspace/jspace/tao_util.cpp
  stanford_wbc/jspace/jspace/Controller.cpp
  stanford_wbc/jspace/jspace/Model.cpp
  stanford_wbc/jspace/jspace/test/util.cpp
  stanford_wbc/jspace/jspace/test/sai_brep.cpp
  stanford_wbc/jspace/jspace/test/sai_brep_parser.cpp
  stanford_wbc/jspace/jspace/test/model_library.cpp
  stanford_wbc/jspace/jspace/test/sai_util.cpp
  stanford_wbc/jspace/jspace/tao_dump.cpp

  stanford_wbc/opspace/src/pseudo_inverse.cpp
  stanford_wbc/opspace/src/task_library.cpp
  stanford_wbc/opspace/src/Skill.cpp
  stanford_wbc/opspace/src/skill_library.cpp
  stanford_wbc/opspace/src/Factory.cpp
  stanford_wbc/opspace/src/Task.cpp
  stanford_wbc/opspace/src/ClassicTaskPostureController.cpp
  stanford_wbc/opspace/src/parse_yaml.cpp
  stanford_wbc/opspace/src/Parameter.cpp
  stanford_wbc/opspace/src/TypeIOTGCursor.cpp
  stanford_wbc/opspace/src/Controller.cpp
  
  src/opspace_param_callbacks.cpp
  )
