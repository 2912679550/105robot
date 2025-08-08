#include "autoPipeSolver.hpp"
#include "ros_topic_channel.hpp"

AutoPipeSolver::AutoPipeSolver(){
    generate_diff_funcs_();
}

AutoPipeSolver::~AutoPipeSolver(){

}

void AutoPipeSolver::set_pipe_params(double l4 , double R , double r){
    params_.l4 = l4;
    params_.R = R;
    params_.r = r;
    ROS_INFO_STREAM("Set pipe parameters: l4=" << l4 << ", R=" << R << ", r=" << r);
    generate_diff_funcs_();
}


SolverState AutoPipeSolver::solve(double theta_deg, double target_v, 
                        float * push_length, float * main_wheel_speed, float * assist_wheel_speed) {
    timer_.reset();
    theta_deg_ = theta_deg;
    theta_rad_ = theta_deg * PI / 180.0; // 转换为弧度
    target_v_ = target_v;
    
    // * 1. 求解此时的beta角度, 并在其内部校验x值
    timer_.get_ms_duration();    // 单位 ms
    SolverState beta_state = solve_beta_();
    float time_cost_beta = timer_.get_ms_duration();    // 单位 ms
    
    if(beta_state == SolverState::BetaSolveFailed){
        std::cout   << RED_STRING << BLOD_STRING
                    << "Beta angle solve failed, theta_deg: " << theta_deg_
                    << RESET_STRING << std::endl;
        return SolverState::BetaSolveFailed;
    }
    else if(beta_state == SolverState::solveEnd){
        std::cout   << GREEN_STRING << BLOD_STRING
                    << "Auto Pipe motion finished , please continue" 
                    << RESET_STRING << std::endl;
        return SolverState::solveEnd;
    }

    // * 2. 基于求解得到的beta角度，计算推杆长度
    double deg_answer = params_.deg_alpha - beta_deg_;
    // length = power(100.4241 , 2) + power(83 , 2) - 2 * 100.4241 * 83 * cos(deg_answer / 180 * pi);
    *push_length = std::sqrt(
                std::pow(body_length_BC, 2) + std::pow(body_length_CE, 2) - 
                2 * body_length_BC * body_length_CE * std::cos(deg_answer / 180.0 * PI)
    );

    // * 3. 基于此时的beta与theta角，计算期望轮速
    timer_.get_ms_duration();    // 单位 ms
    SolverState wheel_speed_state = compute_wheel_speeds_(main_wheel_speed, assist_wheel_speed);
    float time_cost_wheel_speed = timer_.get_ms_duration();
    if(wheel_speed_state == SolverState::WheelSpeedComputeFailed){
        std::cout   << RED_STRING << BLOD_STRING
                    << "Wheel speed compute failed, theta_deg: " << theta_deg_
                    << RESET_STRING << std::endl;
        return SolverState::WheelSpeedComputeFailed;
    }

    // * 4. 至此应该已经全部求解结束，打印结果并返回状态
    std::cout   << GREEN_STRING << BLOD_STRING
                <<  "========== Auto Pipe motion solved ==========" << "\n"
                << " theta_deg: " << theta_deg_ << "\tbeta_deg: " << beta_deg_ << "\n"
                << " push_length: " << *push_length << "\n"
                // << " main_wheel_speed: " << main_wheel_speed[0] << ", " << main_wheel_speed[1] << "\n"
                // << " assist_wheel_speed: " << assist_wheel_speed[0] << ", " << assist_wheel_speed[1] << "\n"
                << " time_cost  beta: " << time_cost_beta << " ms, "
                << "\twheel_speed: " << time_cost_wheel_speed << " ms" << "\n"
                << RESET_STRING << std::endl;
    return SolverState::Normal;
}

SolverState AutoPipeSolver::solve_beta_(){
    double beta_init = 0.0; // 初始猜测
    beta_rad_ = beta_init;  // 设置初始值

    // 动态创建cost function，将当前的theta值作为固定参数
    diff_func_beta_singel_ = new ceres::AutoDiffCostFunction<BETA_THETA_FITTING_FUNC_SINGLE_PARAM, 1, 1>(
        new BETA_THETA_FITTING_FUNC_SINGLE_PARAM(params_, theta_rad_)
    );

    ceres::Problem problem;
    problem.AddResidualBlock(
        diff_func_beta_singel_,
        nullptr,
        &beta_rad_ // 只优化 beta_rad_
    );
    problem.SetParameterLowerBound(&beta_rad_, 0, 0.0); // beta下限
    problem.SetParameterUpperBound(&beta_rad_, 0, PI / 6.0); // beta上限  暂时给为30度

    ceres::Solver::Options options; 
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false; // 输出求解过程
    options.linear_solver_type = ceres::DENSE_QR; // 使用Dense QR求解器
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 转换为度数用于后续计算
    beta_deg_ = beta_rad_ * 180.0 / PI;

    // if( summary.IsSolutionUsable() == true)

    return SolverState::Normal; // 返回正常状态
}

SolverState AutoPipeSolver::compute_wheel_speeds_(float* main_wheel_speed , float * assist_wheel_speed){
    return SolverState::Normal; // 返回正常状态
}

void AutoPipeSolver::generate_diff_funcs_(){
    // 这里暂时不创建cost function，因为theta值在运行时确定
    // 我们将在solve_beta_()函数中动态创建
}


