rosbuild_add_library(observers
        nearest_backward.cc
        nearest_forward.cc
        observer.cc
        QuadrilateralOps.cc
        filter.cc)
target_link_libraries(observers artmap)
