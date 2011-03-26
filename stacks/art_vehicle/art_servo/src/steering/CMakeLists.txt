rosbuild_add_executable(steering steering.cc devsteer.cc testwheel.cc)

# unit tests
rosbuild_add_gtest(test_devsteer test_devsteer.cc devsteer.cc)
