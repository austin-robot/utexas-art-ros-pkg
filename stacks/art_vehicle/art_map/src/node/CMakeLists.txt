rosbuild_add_executable(maplanes mapl.cc)
target_link_libraries(maplanes artmap)

#rosbuild_add_executable(rndf_lanes rndf_lanes.cc)
#target_link_libraries(rndf_lanes artmap)

rosbuild_add_executable(test_lanes test_lanes.cc)
target_link_libraries(test_lanes artmap)

rosbuild_add_executable(getpoints getpoints.cc)
