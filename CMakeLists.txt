include_directories (
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../reflexxes_otg/include
  ../
  )

add_library (opspace SHARED
  opspace.cpp
  Task.cpp
  controller_library.cpp
  )
target_link_libraries (opspace jspace reflexxes_otg)

add_executable (hello_world hello_world.cpp)
target_link_libraries (hello_world opspace)

add_executable (testTask testTask.cpp)
target_link_libraries (testTask opspace jspace_test)
