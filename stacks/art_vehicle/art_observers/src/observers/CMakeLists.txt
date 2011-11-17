rosbuild_add_library(observers
	adjacent_left.cc
	adjacent_right.cc
	filter.cc
        nearest_backward.cc
        nearest_forward.cc
        observer.cc
        QuadrilateralOps.cc
        )
target_link_libraries(observers artmap)
