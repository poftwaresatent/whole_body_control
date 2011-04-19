include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../reflexxes_otg/include
  ../yaml-cpp/include
  include
  )

add_library (opspace SHARED
  src/opspace.cpp
  src/Parameter.cpp
  src/Task.cpp
  src/Factory.cpp
  src/TypeIOTGCursor.cpp
  src/Controller.cpp
  src/controller_library.cpp
  src/task_library.cpp
  src/skill_library.cpp
  src/parse_yaml.cpp
  src/Skill.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg yaml-cpp)

if (HAVE_GTEST)

  add_executable (testTask src/testTask.cpp)
  target_link_libraries (testTask opspace jspace_test gtest pthread)

  add_executable (testFactory src/testFactory.cpp)
  target_link_libraries (testFactory opspace gtest pthread)

endif (HAVE_GTEST)
