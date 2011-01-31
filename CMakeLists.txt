include_directories (
  ../stanford_wbc/jspace
  .
  )

add_library (opspace SHARED
  opspace.cpp
  TaskDescription.cpp
  )
target_link_libraries (opspace jspace)

add_executable (hello_world hello_world.cpp)
target_link_libraries (hello_world opspace)

add_executable (testTaskDescription testTaskDescription.cpp)
target_link_libraries (testTaskDescription opspace)
