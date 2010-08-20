rosbuild_add_executable(commander Blockage.cc command.cc FSM.cc ros_node.cc)
target_link_libraries(commander artnav artmap)

