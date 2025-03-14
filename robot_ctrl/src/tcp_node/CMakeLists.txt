add_executable(tcp_node tcp_server_node.cpp)
add_dependencies(tcp_node robot_ctrl_generate_messages_cpp)
target_link_libraries(tcp_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})