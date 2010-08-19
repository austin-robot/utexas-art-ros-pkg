#rosbuild_add_executable(commander
#  command.cc command_front.cc FSM.cc Blockage.cc)
rosbuild_add_executable(commander FSM.cc)
target_link_libraries(commander logc)

