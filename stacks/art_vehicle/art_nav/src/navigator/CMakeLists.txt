# Controllers still to be ported
#rosbuild_add_executable(navigator
#  avoid.cc
#  do_nothing.cc
#  evade.cc
#  lane_edge.cc
#  parking.cc
#  passing.cc
#  real_zone.cc
#  safety.cc
#  stop.cc
#  voronoi_zone.cc
#  zone.cc)

# currently ported to ROS
rosbuild_add_executable(navigator
  course.cc
  estop.cc
  follow_lane.cc
  follow_safely.cc
  navigator.cc
  obstacle.cc
  queue_mgr.cc
  road.cc
  run.cc
  slow_for_curves.cc
  stop_area.cc
  stop_line.cc
  #uturn.cc
  )
target_link_libraries(navigator artnav artmap)
