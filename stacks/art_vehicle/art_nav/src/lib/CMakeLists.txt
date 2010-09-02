rosbuild_add_library(artnav
  estimate.cc
  FSMstate.cc
  GraphSearch.cc
  Mission.cc
  NavBehaviors.cc
  NavEstopState.cc
  NavRoadState.cc
  )
target_link_libraries(artnav artmap)
