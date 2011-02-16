include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../reflexxes_otg/include
  ../yaml-cpp/include
  include
  )

add_library (opspace SHARED
  src/opspace.cpp
  src/Task.cpp
  src/TaskFactory.cpp
  src/TypeIOTGCursor.cpp
  src/Controller.cpp
  src/controller_library.cpp
  src/task_library.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg yaml-cpp)

add_executable (hello_world src/hello_world.cpp)
target_link_libraries (hello_world opspace)

if (HAVE_GTEST)
  add_executable (testTask src/testTask.cpp)
  target_link_libraries (testTask opspace jspace_test gtest)
endif (HAVE_GTEST)

add_executable (testTaskFactory src/testTaskFactory.cpp)
target_link_libraries (testTaskFactory opspace)
