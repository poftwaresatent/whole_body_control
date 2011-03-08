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
  src/ControllerNG.cpp
  src/controller_library.cpp
  src/task_library.cpp
  src/behavior_library.cpp
  src/other_controllers.cpp
  src/parse_yaml.cpp
  src/Behavior.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg yaml-cpp)

add_executable (hello_world src/hello_world.cpp)
target_link_libraries (hello_world opspace)

if (HAVE_GTEST)
  add_executable (testTask src/testTask.cpp)
  target_link_libraries (testTask opspace jspace_test gtest)
  add_executable (testFactory src/testFactory.cpp)
  target_link_libraries (testFactory opspace gtest)
endif (HAVE_GTEST)
