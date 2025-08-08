find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pipetest 
    autoPipeSolver.cpp
    pipeTest.cpp
    ../Timer/Timer.cpp
)
target_link_libraries(pipetest ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN3_LIBRARIES})
