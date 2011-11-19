rosbuild_add_executable(points_on_road points_on_road.cc)
target_link_libraries(points_on_road artmap)

# observers node
rosbuild_add_executable(observers_node
        observers_node.cc
        lane_observations.cc)
target_link_libraries(observers_node observers artmap)
