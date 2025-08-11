cmake_minimum_required(VERSION 3.0.2)
project(robot_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    #serial_steer
    serial
    message_generation
    camera_ctrl
    # opencv2
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
LINK_DIRECTORIES("/usr/local/lib/")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    robot_motion_val.msg
    tcp_motion_cmd.msg
    # 单侧控制板
    single_side_cmd.msg
    single_side_val.msg
    # 中置推杆控制板
    push_board_cmd.msg
    push_board_val.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  sensor_msgs std_srvs  camera_ctrl
)

include_directories(
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

add_subdirectory(src/tcp_node)
add_subdirectory(src/pipe_solver)

add_executable(robot_node 
    src/robot_node/robot_node.cpp
    src/robot_node/robot_run.cpp
    src/robot_node/public.cpp
    src/robot_node/imu_handler.cpp
    src/robot_node/pid.cpp
    src/robot_node/board_ctrl.cpp
    src/robot_node/robot_params.cpp
    src/Timer/Timer.cpp
    ${PIPE_SOLVER_SOURCES}  # 来自子路径
)
add_dependencies(robot_node robot_ctrl_generate_messages_cpp)
target_link_libraries(robot_node 
${catkin_LIBRARIES} 
${Boost_LIBRARIES} 
${OpenCV_LIBS}
${CERES_LIBRARIES}
)


