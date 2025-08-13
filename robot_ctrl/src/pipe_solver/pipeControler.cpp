#include "pipeControler.hpp"
#include "ros_topic_channel.hpp"

PipeController::PipeController(){
    solver_ = new AutoPipeSolver(); // 初始化管道求解器
    set_geometry_params_(pipe_R_, pipe_r_); // 设置管道几何参数
}

PipeController::~PipeController(){

}

/**
 * @brief 设置自动进弯的起始位置，便于类自动记忆
 * 
 * @param odom_aix 当前位置的主动轮轴向里程计坐标
 * @param odom_cir 当前位置的主动轮周向里程计坐标
 * @param imu_quat 当前位置的IMU四元数数值（指针）
 * @return true 
 * @return false 
 */
bool PipeController::set_start_(float odom_aix , float odom_cir , tf::Quaternion* imu_quat){
    is_started_ = true; // 设置起始位置标志位为true
    start_odom_aix_ = odom_aix; // 设置起始位置轴
    start_odom_cir_ = odom_cir; // 设置起始位置周向
    if(imu_quat != nullptr)  start_imu_quat_ = *imu_quat; // 设置起始位置IMU四元数
    return true; // 返回true表示设置成功
}

bool PipeController::auto_in_pipe_(float odom_aix , float odom_cir , tf::Quaternion* cur_quat, 
                                    float target_v, bool printFlag){
    if(is_started_ == false){
        if(printFlag == true) 
        std::cout   << RED_STRING << UNDERLINE_STRING
                    << "PipeController: Please set start position first!"
                    << RESET_STRING << std::endl;
        push_length_ = body_angle_push_baseline; // 如果未设置起始位置，则使用默认推杆长度
        main_wheel_speed_[0] = 0.0f; // 主动轮
        main_wheel_speed_[1] = 0.0f; // 主动轮周向速度
        assist_wheel_speed_[0] = 0.0f; // 辅助
        assist_wheel_speed_[1] = 0.0f; // 辅助轮周向速度
        return false; // 如果未设置起始位置，则返回false
    }

    float dis_aix = odom_aix - start_odom_aix_; // 计算轴向位移
    float theta_deg = dis_aix / (pipe_R_ + pipe_r_) * 180.0 / PI; // 计算当前的theta角度

    SolverState result_state =  solver_->solve(theta_deg , target_v,
                                    &push_length_,  
                                    main_wheel_speed_, 
                                    assist_wheel_speed_, 
                                    printFlag); // 调用求解器进行求解
    if(result_state == SolverState::BetaSolveFailed || 
    result_state == SolverState::WheelSpeedComputeFailed ){
        // 求解异常或进弯结束 ， 则将速度设为0
        main_wheel_speed_[0] = 0.0f; // 主动轮
        main_wheel_speed_[1] = 0.0f; // 主动轮
        assist_wheel_speed_[0] = 0.0f; // 辅助
        assist_wheel_speed_[1] = 0.0f; // 辅助
        return false;
    }else{
        if (result_state == SolverState::solveEnd) solve_end_ = true;
        else solve_end_ = false; // 设置求解结束状态
        return true;
    }
}

bool PipeController::auto_out_pipe_(float odom_aix , float odom_cir , tf::Quaternion* cur_quat, 
                                    float target_v, bool printFlag){
    if(is_started_ == false){
        if(printFlag == true) 
        std::cout   << RED_STRING << UNDERLINE_STRING
                    << "PipeController: Please set start position first!"
                    << RESET_STRING << std::endl;
        push_length_ = body_angle_push_baseline; // 如果未设置起始位置，则使用默认推杆长度
        main_wheel_speed_[0] = 0.0f; // 主动轮
        main_wheel_speed_[1] = 0.0f; // 主动轮周向速度
        assist_wheel_speed_[0] = 0.0f; // 辅助
        assist_wheel_speed_[1] = 0.0f; // 辅助轮周向速度
        return false; // 如果未设置起始位置，则返回false
    }

    float dis_aix = odom_aix - start_odom_aix_; // 计算轴向位移
    float theta_deg = dis_aix / (pipe_R_ + pipe_r_) * 180.0 / PI; // 计算当前的theta角度
    float mirror_theta_deg = solver_->max_theta_deg_ - theta_deg; // 计算镜像角度
    // * 注意这里由于是出弯， 所以需要将期望速度先取反，使用镜像角度，最后使用时再将输出的速度取反
    SolverState result_state = solver_->solve(mirror_theta_deg , -target_v,
                                    &push_length_, 
                                    main_wheel_speed_, 
                                    assist_wheel_speed_, 
                                    printFlag); // 调用求解器进行求解
    if(result_state == SolverState::BetaSolveFailed || 
    result_state == SolverState::WheelSpeedComputeFailed ){
        // 求解异常或进弯结束 ， 则将速度设为0
        main_wheel_speed_[0] = 0.0f; // 主动轮
        main_wheel_speed_[1] = 0.0f; // 主动轮
        assist_wheel_speed_[0] = 0.0f; // 辅助
        assist_wheel_speed_[1] = 0.0f; // 辅助
        return false;
    }else{
        // 将速度取反
        main_wheel_speed_[0] = -main_wheel_speed_[0]; // 主
        main_wheel_speed_[1] = -main_wheel_speed_[1]; // 主动轮周向速度
        assist_wheel_speed_[0] = -assist_wheel_speed_[0]; // 辅助轮
        assist_wheel_speed_[1] = -assist_wheel_speed_[1]; // 辅助轮周向速度
        if (result_state == SolverState::solveEnd) solve_end_ = true;
        else solve_end_ = false; // 设置求解结束状态
        return true;
    }
}

/**
 * @brief 配置自动过弯控制器的内置几何参数
 * 
 * @param R         当前管道转弯半径
 * @param r         当前管道半径， 目前采用直接将管道半径生成l4尺寸参数
 * @return bool值，缺省
 */
bool PipeController::set_geometry_params_(float R, float r){
    if(solver_ == nullptr) return false; // 如果求解器未初始化，返回false
    solver_ ->set_pipe_params(R, r); // 设置管道参数
    pipe_R_ = R; // 设置管道转弯半径
    pipe_r_ = r; // 设置管道半径
    return true;
}
