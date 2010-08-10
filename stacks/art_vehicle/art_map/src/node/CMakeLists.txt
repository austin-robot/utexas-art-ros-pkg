rosbuild_add_executable(maplanes mapl.cc)
link_directories(maplanes ${PROJECT_NAME}/lib)
target_link_libraries(maplanes artmap)

#rosbuild_add_executable(rndf_lanes rndf_lanes.cc)
#link_directories(rndf_lanes ${PROJECT_NAME}/lib)
#target_link_libraries(rndf_lanes artmap)

rosbuild_add_executable(test_lanes test_lanes.cc)
link_directories(test_lanes ${PROJECT_NAME}/lib)
target_link_libraries(test_lanes artmap)
