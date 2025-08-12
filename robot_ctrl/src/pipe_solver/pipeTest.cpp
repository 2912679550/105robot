#include "autoPipeSolver.hpp"
#include "pipeControler.hpp"

int main(int argc, char **argv)
{
    AutoPipeSolver solver;
    PipeController controller;

    float push_length = 0.0f;
    float main_wheel_speed[2] = {0.0f, 0.0f};
    float assist_wheel_speed[2] = {0.0f, 0.0f};
    // solver.set_pipe_params( 300.0, 110.0);
    solver.solve(40.7028, 0.02,
                &push_length, main_wheel_speed, assist_wheel_speed);

    controller.set_geometry_params_(300.0, 180.0);
    std::cout<<"debug 1"<< std::endl;
    controller.set_start_(0.0f, 0.0f, nullptr); // 设置起始位置
    controller.auto_in_pipe_(200.0f, 0.0f, nullptr, 0.02f, true); // 自动进入管道
    controller.auto_in_pipe_(230.0f, 0.0f, nullptr, 0.02f, true); // 自动进入管道

    return 0; 
}




