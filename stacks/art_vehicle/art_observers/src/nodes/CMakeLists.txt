rosbuild_add_executable(points_on_road points_on_road.cc)
target_link_libraries(points_on_road artmap)

# initial version -- to be replaced
rosbuild_add_executable(map_grid
        map_grid_node.cc
        map_grid.cc
        lane_observer.cc)
target_link_libraries(map_grid observers artmap)

# redesigned version
rosbuild_add_executable(observers_node
        observers_node.cc
        lane_observations.cc)
target_link_libraries(observers_node observers artmap)
