add_executable(robot_node 
robot_node.cpp
robot_run.cpp
)
add_dependencies(robot_node robot_ctrl_generate_messages_cpp)
target_link_libraries(robot_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})