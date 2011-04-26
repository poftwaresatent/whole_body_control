include_directories (
  ../stanford_wbc/opspace/include
  ../stanford_wbc/jspace
  ../stanford_wbc/tao
  ../stanford_wbc/3rdparty/reflexxes_otg/include
  ../stanford_wbc/3rdparty/yaml-cpp/include
  .
  )

add_library (uta_opspace SHARED
  DelayHistogram.cpp
  strutil.cpp
  ControllerNG.cpp
  HelloGoodbyeSkill.cpp
  )
target_link_libraries (uta_opspace opspace jspace reflexxes_otg yaml-cpp)
