#include "autoPipeSolver.hpp"
#include "ros_topic_channel.hpp"

AutoPipeSolver::AutoPipeSolver(){
    generate_diff_funcs_();
    get_max_theta_deg_(); // 计算最大theta角度
}

AutoPipeSolver::~AutoPipeSolver(){

}

void AutoPipeSolver::set_pipe_params(double R , double r){
    params_.l4 = params_.l3 + 1.5 * r;
    params_.R = R;
    params_.r = r;
    ROS_INFO_STREAM("Set pipe parameters: l4=" << params_.l4 << ", R=" << R << ", r=" << r);
    generate_diff_funcs_();
    get_max_theta_deg_(); // 计算最大theta角度
}

void AutoPipeSolver::get_max_theta_deg_(){
    // * 计算此时的最大theta角度
    auto theta_max_func = new ceres::AutoDiffCostFunction<THETA_MAX_FUNC, 1, 1>(
        new THETA_MAX_FUNC(params_)
    );
    ceres::Problem problem;
    double theta_init = 0.2; // 初始猜测
    problem.AddResidualBlock(
        theta_max_func,
        nullptr,
        &theta_init // 只优化 theta_init
    );
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // 使用稠密
    options.max_num_iterations = 100; // 最大迭代次数
    options.minimizer_progress_to_stdout = false; // 不输出迭代过程
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if(summary.IsSolutionUsable()){
        max_theta_deg_ = theta_init * 180.0 / PI; // 转换为度
        ROS_INFO_STREAM("Max theta_deg: " << max_theta_deg_);
    }

    double deg_answer = params_.deg_alpha - max_theta_deg_ / 2.0;
    // length = power(100.4241 , 2) + power(83 , 2) - 2 * 100.4241 * 83 * cos(deg_answer / 180 * pi);
    min_push_length_ = std::sqrt(
                std::pow(body_length_BC, 2) + std::pow(body_length_CE, 2) - 
                2 * body_length_BC * body_length_CE * std::cos(deg_answer / 180.0 * PI)
    );
    ROS_INFO_STREAM("Min push_length: " << min_push_length_);
}

/**
 * @brief 核心求解函数，根据输入的theta角度值以及期望速度，解算此时理论推杆长度与轮速
 * 
 * @param theta_deg 此时theta角度值，单位为度
 * @param target_v  期望的主动轮线速度，单位为m/s
 * @param push_length 输出的期望推杆长度，单位为mm
 * @param main_wheel_speed 输出的主动轮速度，单位为m/s, [轴向速度, 周向速度]
 * @param assist_wheel_speed  输出的辅助轮速度，单位为m/s, [轴向速度, 周向速度]
 * @param printFlag  是否打印求解过程中的信息
 * @return SolverState 
 */
SolverState AutoPipeSolver::solve(double theta_deg, double target_v, 
                        float * push_length, float * main_wheel_speed, float * assist_wheel_speed, bool printFlag) {
    timer_.reset();
    theta_deg_ = theta_deg;
    theta_rad_ = theta_deg * PI / 180.0; // 转换为弧度
    target_v_ = target_v; 
    
    // * 0. 新增，如果theta角度超过最大值，则直接判定机器人匀速运动
    if(theta_deg_ > max_theta_deg_ * 0.92){
        *push_length = min_push_length_;
        main_wheel_speed[0] = target_v_;
        main_wheel_speed[1] = 0.0; // 主动轮轴向速度为0
        assist_wheel_speed[0] = target_v * (params_.R - 0.5 * params_.r) / (params_.r + params_.R);
        assist_wheel_speed[1] = 0.0; // 辅助轮轴向速度为0
        if(printFlag == true){
            std::cout   << YELLOW_STRING << BLOD_STRING
            <<  "========== Auto into/out Pipe motion Finished !==========" << "\n"
            << RESET_STRING << GREEN_STRING << BLOD_STRING
            << " theta_deg: " << theta_deg_ << "\n"
            << " push_length: " << *push_length << "\n"
            << " main_wheel_speed: " << main_wheel_speed[0] << ", " << main_wheel_speed[1] << "\n"
            << " assist_wheel_speed: " << assist_wheel_speed[0] << ", " << assist_wheel_speed[1] << "\n"
            << RESET_STRING << std::endl;
        }
        return SolverState::solveEnd; // 认为求解结束
    }
    else if(theta_deg_ < 0.0f){         // 自动出弯末期会用到
        *push_length = body_angle_push_baseline; // 如果theta小于0，则使用默认推杆长度
        main_wheel_speed[0] = target_v; // 主动轮
        main_wheel_speed[1] = 0.0; // 主动轮轴向速度为0
        assist_wheel_speed[0] = target_v;
        assist_wheel_speed[1] = 0.0; // 辅助轮轴向速度为0
        if(printFlag == true){
            std::cout   << YELLOW_STRING << BLOD_STRING
                        <<  "========== Auto into/out Pipe motion Finished !==========" << "\n"
                        << RESET_STRING << GREEN_STRING << BLOD_STRING
                        << " theta_deg: " << theta_deg_ << "\n"
                        << " push_length: " << *push_length << "\n"
                        << " main_wheel_speed: " << main_wheel_speed[0] << ", " << main_wheel_speed[1] << "\n"
                        << " assist_wheel_speed: " << assist_wheel_speed[0] << ", " << assist_wheel_speed[1] << "\n"
                        << RESET_STRING << std::endl;
        }
        return SolverState::solveEnd; // 认为求解结束
    }

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
    if(printFlag == false) return SolverState::Normal;                                                      
    std::cout   << GREEN_STRING << BLOD_STRING
                <<  "========== Auto Pipe motion solved ==========" << "\n"
                << " theta_deg: " << theta_deg_ << "\tbeta_deg: " << beta_deg_ << "\n"
                << " push_length: " << *push_length << "\tdis_x: " << cur_dis_x_ << "\n"
                << " main_wheel_speed: " << main_wheel_speed[0] << ", " << main_wheel_speed[1] << "\n"
                << " assist_wheel_speed: " << assist_wheel_speed[0] << ", " << assist_wheel_speed[1] << "\n"
                << " time_cost  beta: " << time_cost_beta << " ms, "
                << "\twheel_speed: " << time_cost_wheel_speed << " ms" << "\n"
                << RESET_STRING << std::endl;
    return SolverState::Normal;
}

SolverState AutoPipeSolver::solve_beta_(){
    double beta_init = 0.2; // 初始猜测
    beta_rad_ = beta_init;  // 设置初始值

    // 动态创建cost function，将当前的theta值作为固定参数
    // diff_func_beta_singel_ = new ceres::AutoDiffCostFunction<BETA_THETA_FITTING_FUNC_SINGLE_PARAM, 1, 1>(
    diff_func_beta_singel_ = new ceres::NumericDiffCostFunction<BETA_THETA_FITTING_FUNC_SINGLE_PARAM, ceres::CENTRAL , 1, 1>(
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
    options.max_num_iterations = 30;
    options.minimizer_progress_to_stdout = false; // 输出求解过程
    options.linear_solver_type = ceres::DENSE_QR; // 使用Dense QR求解器
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 转换为度数用于后续计算
    beta_rad_   = beta_rad_ + 0.00; // 修正beta_rad_的值 , 防止后边求不出偏导
    beta_deg_ = beta_rad_ * 180.0 / PI;

    // 计算一下测试的x长度
    cur_dis_x_ =  dis_x_func_(theta_rad_ , beta_rad_);
    if(cur_dis_x_ < 5.0){
        return SolverState::solveEnd; // 如果x小于5mm，认为求解结束
    }

    if( summary.IsSolutionUsable() == true)
        return SolverState::Normal; // 返回正常状态
    else
        return SolverState::BetaSolveFailed; // 返回beta求解失败状态
}

SolverState AutoPipeSolver::compute_wheel_speeds_(float* main_wheel_speed , float * assist_wheel_speed){
    // * 计算直角坐标系下的主动轮与管道接触点速度
    double theta_dot = target_v_ / (params_.R + params_.r);
    double dxm_dtheta = ceres::cos(theta_rad_) * (params_.r + params_.R);
    double dym_dtheta =-ceres::sin(theta_rad_) * (params_.r + params_.R);
    double vx_m = dxm_dtheta * theta_dot;
    double vy_m = dym_dtheta * theta_dot;

    // * 求解f函数关于beta和theta两个参数的偏导数
    // 修正：创建正确格式的参数数组
    const double* parameters[2] = {&beta_rad_, &theta_rad_};
    double residual[1];  // 残差数组（根据你的cost function输出维度）
    // 为每个参数块分配雅可比矩阵内存
    double df_dbeta[1];   // 对beta的偏导数
    double df_dtheta[1];  // 对theta的偏导数
    double* jacobians[2]= {df_dbeta, df_dtheta};
    // 调用Evaluate函数
    bool success = theta_beta_func_->Evaluate(parameters, residual, jacobians);
    // std::cout << "theta_beta_func_ Evaluate success: " << success << std::endl;
    // std::cout << "df_dbeta: " << df_dbeta[0] << ", df_dtheta: " << df_dtheta[0] << std::endl;
    // 进一步求解得到beta_dot
    double dbeta_dtheta = -df_dtheta[0] / df_dbeta[0];
    double beta_dot = dbeta_dtheta * theta_dot;

    // * 求解xq关于beta和theta的偏导数，yq关于beta的偏导数
    double xq_residual[1];
    double yq_residual[1];
    
    double dxq_dbeta[1], dxq_dtheta[1];
    double* xq_jacobians[2] = {dxq_dbeta, dxq_dtheta};

    double dyq_dbeta[1];
    double* yq_jacobians[1] = {dyq_dbeta};

    // 计算xq关于beta和theta的偏导数
    bool xq_success = xq_func_->Evaluate(parameters, xq_residual, xq_jacobians);
    // std::cout << "xq_func_ Evaluate success: " << xq_success << std::endl;
    const double* yq_parameters[1] = {&beta_rad_};
    bool yq_success = yq_func_->Evaluate(yq_parameters, yq_residual, yq_jacobians);
    // std::cout << "yq_func_ Evaluate success: " << yq_success << std::endl;

    // * 计算辅助轮点的直角坐标系下速度
    double vx_q = dxq_dbeta[0] * beta_dot + dxq_dtheta[0] * theta_dot;
    double vy_q = dyq_dbeta[0] * beta_dot;

    // * 将直角速度转换为舵轮轴向与周向速度
    main_wheel_speed[0] = vx_m * ceres::cos(theta_rad_) - vy_m * ceres::sin(theta_rad_);
    main_wheel_speed[1] = 0.0; // 主动轮的周向速度为0
    assist_wheel_speed[0] = vx_q * ceres::cos(2.0 * beta_rad_) - vy_q * ceres::sin(2.0 * beta_rad_);
    assist_wheel_speed[1] =-vx_q * ceres::sin(2.0 * beta_rad_) - vy_q * ceres::cos(2.0 * beta_rad_);
    return SolverState::Normal; // 返回正常状态
}

void AutoPipeSolver::generate_diff_funcs_(){
    theta_beta_func_ = new ceres::AutoDiffCostFunction<BETA_THETA_FITTING_FUNC, 1, 1, 1>(
        new BETA_THETA_FITTING_FUNC(params_)
    );
    xq_func_ = new ceres::AutoDiffCostFunction<X_Q_FUNC , 1, 1 , 1>(
        new X_Q_FUNC(params_)
    );
    yq_func_ = new ceres::AutoDiffCostFunction<Y_Q_FUNC , 1, 1>(
        new Y_Q_FUNC(params_)
    );
}

double AutoPipeSolver::dis_x_func_(double theta_r , double beta_r){
    // x = (l1 + l2 / (2 * cos(b) )) * (1 + cos(2 * b)) - (l3 - rw) * sin(2 * b) - sin(t) * (R + r + rw);
    double term_x1 = (params_.l1 + params_.l2 / (2.0 * ceres::cos(beta_r))) * (1.0 + ceres::cos(2 * beta_r));
    double term_x2 = (params_.l3 - params_.rw) * ceres::sin(2.0* beta_r);
    double term_x3 = ceres::sin(theta_r) * (params_.R + params_.r + params_.rw);
    double x = term_x1 - term_x2 - term_x3;
    return x;
}
