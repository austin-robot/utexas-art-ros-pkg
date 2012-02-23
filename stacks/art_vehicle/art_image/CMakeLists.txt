cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# unit test: parse check of all the launch/*.launch files
rosbuild_add_roslaunch_check(launch)
