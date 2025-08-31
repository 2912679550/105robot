# find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# 获取当前CMakeList文件绝对路径



set(PIPE_SOLVER_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/autoPipeSolver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/pipeControler.cpp
    PARENT_SCOPE
)

# add_executable(pipetest 
#     autoPipeSolver.cpp
#     pipeControler.cpp
#     pipeTest.cpp
#     ../Timer/Timer.cpp
# )
# target_link_libraries(pipetest ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN3_LIBRARIES})
