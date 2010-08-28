#rosbuild_add_executable(navigator
#  avoid.cc
#  course.cc
#  do_nothing.cc
#  estop.cc
#  evade.cc
#  follow_lane.cc
#  follow_safely.cc
#  lane_edge.cc
#  lane_heading.cc
#  navigator.cc
#  obstacle.cc
#  passing.cc
#  queue_mgr.cc
#  real_zone.cc
#  parking.cc
#  road.cc
#  run.cc
#  safety.cc
#  slow_for_curves.cc
#  stop.cc
#  stop_area.cc
#  stop_line.cc
#  uturn.cc
#  voronoi_zone.cc
#  zone.cc)
rosbuild_add_executable(navigator
  course.cc
  estop.cc
  navigator.cc
  queue_mgr.cc
  run.cc
  )
target_link_libraries(navigator artnav artmap)
