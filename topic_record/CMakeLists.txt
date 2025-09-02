cmake_minimum_required(VERSION 3.0.2)
project(topic_record)

# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS roscpp robot_ctrl)
find_package(yaml-cpp REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES topic_record
    CATKIN_DEPENDS roscpp robot_ctrl
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/topic_record.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
    ${PROJECT_NAME}_node
    src/topic_record_node.cpp
    src/Timer.cpp
    src/record.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    yaml-cpp
)

