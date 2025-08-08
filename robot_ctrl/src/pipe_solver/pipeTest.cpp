#include "autoPipeSolver.hpp"

int main(int argc, char **argv)
{
    AutoPipeSolver solver;

    float push_length = 0.0f;
    solver.solve(15.0 , 0.02, 
                    &push_length, nullptr, nullptr);

    return 0;
}