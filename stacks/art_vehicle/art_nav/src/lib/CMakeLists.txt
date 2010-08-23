rosbuild_add_library(artnav
  FSMstate.cc
  GraphSearch.cc
  lasers.cc
  Mission.cc
  NavBehaviors.cc
  NavEstopState.cc
  NavRoadState.cc
  odometry.cc
  )
target_link_libraries(artnav artmap)
