#include "autoPipeSolver.hpp"

int main(int argc, char **argv)
{
    AutoPipeSolver solver;

    float push_length = 0.0f;
    float main_wheel_speed[2] = {0.0f, 0.0f};
    float assist_wheel_speed[2] = {0.0f, 0.0f};
    solver.set_pipe_params(280.12, 300.0, 110.0);
    solver.solve(24.7028, 0.02,
                &push_length, main_wheel_speed, assist_wheel_speed);

    return 0;
}