#include "board_ctrl.hpp"
#include "robot_params.hpp"
#include "tf/transform_datatypes.h"

// ! ========================== micro odom ctrl ===========================
// ! ========================== micro odom ctrl ===========================
MICRO_ODOM::MICRO_ODOM(){
    // 初始化里程计数据
    for (int i = 0; i < 3; i++){
        pre_position_[i] = Eigen::Vector2f(0.0f, 0.0f);  // 初始化为零
        cur_position_[i] = Eigen::Vector2f(0.0f, 0.0f);  // 初始化为零
    }
}

MICRO_ODOM::~MICRO_ODOM(){
    // 清理里程计数据
    for (int i = 0; i < 3; i++){
        pre_position_[i].setZero();  // 清零
        cur_position_[i].setZero();  // 清零
    }
}

void MICRO_ODOM::reset(){
    // 重置里程计标志
    resetFlag = true;
}

void MICRO_ODOM::set_cur_val(float axis, float cir){
    odom_axis = axis;  // 设置当前轴向里程计值
    odom_cir = cir;    // 设置当前周向里程计值
}

void MICRO_ODOM::update(STM_ROBOT_VAL_TYPE *val_data, bool printFlag ){
    if(resetFlag == true){
        odom_axis = 0.0f;  // 重置轴向里程计
        odom_cir = 0.0f;   // 重置周向里程计
        resetFlag = false;  // 重置标志位
        for (int i = 0; i < 3; i++){
            pre_position_[i].setZero();  // 清零上一次的位置
            cur_position_[i].setZero();   // 清零当前的位置
        }
    }
    else{
        Eigen::Vector2f delta[3];
        for (int i = 0; i < 3; i++)
        {
            pre_position_[i] = cur_position_[i];  // 保存上一次的位置
            cur_position_[i] = Eigen::Vector2f(val_data->odom_axis[i], val_data->odom_cir[i]);  // 更新当前的位置
            delta[i] = cur_position_[i] - pre_position_[i];  // 计算位置增量
        }
        // // 求出到三个增量delta之间总距离最小的二维向量
        // Eigen::Vector2f total_delta = Eigen::Vector2f(0.0f, 0.0f);
        // 使用平均距离作为单边运动的增量距离
        Eigen::Vector2f meanDelta = (delta[0] + delta[1] + delta[2]) / 3.0f;  // 计算平均增量
        if(startOdom == true){
            odom_axis += meanDelta.x();  // 更新轴向里程计
            odom_cir += meanDelta.y();    // 更新周向里程计
        }
    }
}

// ! ========================== single side ctrl ===========================
// ! ========================== single side ctrl ===========================

SINGLE_SIDE_CTRL::SINGLE_SIDE_CTRL(std::string cmd_topic, std::string val_topic, ros::NodeHandle *nh)
{
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
    }else{
        nh_ = nh;
    }
    cmd_pub_ = nh_->advertise<ROBOT_STM_CMD_TYPE>(cmd_topic, 1);
    val_sub_ = nh_->subscribe(val_topic, 1, &SINGLE_SIDE_CTRL::val_callback, this);
    std::cout<< GREEN_STRING << BLOD_STRING << "single side ctrl node start\n" 
            << " cmd_topic: " << cmd_topic << "\n"
            << " val_topic: " << val_topic
            << RESET_STRING << std::endl;

    // 初始化控制指令
    for (int i = 0; i < 3;i++){
        cmd_data_.dir_steer_state[i] = steerState::STOP;  // 舵轮当前的工作状态
        cmd_data_.dir_steer_dir[i] = 0.5 * PI;  // 舵轮舵向的角度
        cmd_data_.dir_steer_vel[i] = 0.0f;  // 舵轮舵向的速度
        cmd_data_.dir_spring_length = 1.0f;  // 松开
    }
    odom_handler_ = new MICRO_ODOM();  // 创建里程计处理类实例
    tight_timer_ = new MYTIMER();  // 创建定时器处理类实例
    tight_timer_->reset();
    odom_handler_->reset(); // 重置里程计数据
}

void SINGLE_SIDE_CTRL::create_imu(std::string imu_topic , int imu_id){
    imu_handler_ = new IMU_HANDLER(imu_topic, nh_);
    if(imu_id == IMU_ID::FRONT){
        // 根据3*3数组创建旋转矩阵
        imu_id_ = imu_id;
        imu_handler_->imu_robot_matrix = new tf::Matrix3x3(IMU_FRONT_ROTATE[0][0], IMU_FRONT_ROTATE[0][1], IMU_FRONT_ROTATE[0][2],
                                                           IMU_FRONT_ROTATE[1][0], IMU_FRONT_ROTATE[1][1], IMU_FRONT_ROTATE[1][2],
                                                           IMU_FRONT_ROTATE[2][0], IMU_FRONT_ROTATE[2][1], IMU_FRONT_ROTATE[2][2]);
    }
    else if(imu_id == IMU_ID::BACK){
        // 根据3*3数组创建旋转矩阵
        imu_id_ = imu_id;
        imu_handler_-> imu_robot_matrix = new tf::Matrix3x3(IMU_BACK_ROTATE[0][0], IMU_BACK_ROTATE[0][1], IMU_BACK_ROTATE[0][2],
                                                        IMU_BACK_ROTATE[1][0], IMU_BACK_ROTATE[1][1], IMU_BACK_ROTATE[1][2],
                                                        IMU_BACK_ROTATE[2][0], IMU_BACK_ROTATE[2][1], IMU_BACK_ROTATE[2][2]);
    }
    else{
        std::cout << RED_STRING << "imu id error" << RESET_STRING << std::endl;
        return;
    }

    PID_PARAM pid_params;
    pid_params.p = imu_pitch_P[imu_id_];
    pid_params.i = imu_pitch_I[imu_id_];
    pid_params.d = imu_pitch_D[imu_id_];
    pid_params.Iband = imu_pitch_Iband[imu_id_];
    pid_params.outMin = imu_pitch_outRange[0];
    pid_params.outMax = imu_pitch_outRange[1];
    pid_params.outIMin = imu_pitch_outRange[0];
    pid_params.outIMax = imu_pitch_outRange[1];
    pid_handler_ = new Pid(&pid_params);

}

SINGLE_SIDE_CTRL::~SINGLE_SIDE_CTRL(){
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}

void SINGLE_SIDE_CTRL::single_side_ctrl(){
    // 单侧控制逻辑
    pre_tightFlag_ = cur_tightFlag_;  // 保存上一次的夹紧状态
    if(tarTightFlag_ == true){
        // 如果当前目标状态为夹紧，则开始判断是否到达夹紧阈值
        if(cmd_data_.dir_spring_length -  val_data_.cur_spring_length[0] > -0.5f
            && cmd_data_.dir_spring_length -  val_data_.cur_spring_length[1] > -0.5f){
            // 如果夹紧差长度小于0.5f，则认为已经夹紧
            cur_tightFlag_ = true;  // 更新当前夹紧状态
            // 如果上一次夹紧状态为未夹紧，则重置定时器
            if(pre_tightFlag_ == false) tight_timer_->reset();
        }else{
            cur_tightFlag_ = false;  // 否则认为未夹紧
            tight_timer_->reset();  // 重置定时器
        }
    }else{
        // 如果当前目标状态为松开，则认为未夹紧
        cur_tightFlag_ = false;
        tight_timer_->reset();  // 重置定时器
    }
}

void SINGLE_SIDE_CTRL::pub_cmd(){
    if(steer_state_ == steerState::NORMAL) {
        // 如果当前是正常状态，迭代计算舵轮状态，可能需要姿态矫正的参与
        set_steer(steerState::NORMAL , tar_v_aix_ , tar_v_cir_);  
    }
    cmd_pub_.publish(cmd_data_);
}

void SINGLE_SIDE_CTRL::val_callback(const STM_ROBOT_VAL_CPTR &msg){
    if(msg != nullptr)
    val_data_ = *msg;
    // 里程计会自动接收数据并更新，但会有标志位控制其输出是否会累加
    if(odom_handler_ != nullptr) odom_handler_->update(&val_data_, false);  // 更新里程计数据，不打印
}

void SINGLE_SIDE_CTRL::set_tight(bool tightFlag){
    cmd_data_.dir_spring_length = tightFlag ? 20.0f : 1.0f;
    if(cmd_data_.dir_spring_length < 10.0f){
        tarTightFlag_ = false;  // 如果长度小于10.0f，认为是松开状态
    }else{
        tarTightFlag_ = true;  // 否则认为是夹紧状态
    }
}

void SINGLE_SIDE_CTRL::set_tight(float length){
    cmd_data_.dir_spring_length = length;
    if(cmd_data_.dir_spring_length < 10.0f){
        tarTightFlag_ = false;  // 如果长度小于10.0f，认为是松开状态
    }else{
        tarTightFlag_ = true;  // 否则认为是夹紧状态
    }
}

void SINGLE_SIDE_CTRL::set_angle(float angle){
    cmd_data_.dir_arm_angle[0] = 180.0 - angle;
    cmd_data_.dir_arm_angle[1] = 180.0 - angle;
}

void SINGLE_SIDE_CTRL::set_steer(steerState stateIn , float v_aix , float v_cir){
    steer_state_ = stateIn;  // 更新当前舵轮的工作状态
    tar_v_aix_ = v_aix;  // 更新目标轴向速度
    tar_v_cir_ = v_cir;  // 更新目标周向速度
    float pid_out = 0;
    // v_aix 为轴向速度，v_cir为周向速度，周向速度的正方向对应于舵轮舵向的0弧度处
    if(stateIn == steerState::RESET || stateIn == steerState::STOP){
        for (int i = 0; i < 3;i++){
        cmd_data_.dir_steer_state[i] = stateIn;
        // 配置但不使用
        cmd_data_.dir_steer_dir[i] = 0.5 * PI;
        cmd_data_.dir_steer_vel[i] = 0.0f;
        // if(singleSideFixed == true && stateIn == steerState::STOP){
        //     IMU_POSE aixs_err;
        //     imu_handler_->get_aixs_err(&aixs_err, true);  // 获取IMU的姿态误差
        //     // 计算PID控制器的输出
        //     float pid_out = pid_handler_->Tick(aixs_err.pitch , true);
        //     if( i == 2 ) v_aix += pid_out;  // 如果是主动驱动轮，误差为正时应该增大轴向速度
        //     else v_aix -= pid_out;  // 辅助驱动轮，误差为正时应该减小轴向速度
        // }
        }
    }
    else if (stateIn = steerState::NORMAL)
    {
        if (singleSideFixed == true)
        {
            // 如果当前是单侧固定状态，需要使用PID控制器调整单侧姿态
            IMU_POSE aixs_err;
            imu_handler_->get_aixs_err(&aixs_err, true); // 获取IMU的姿态误差
            // 计算PID控制器的输出
            pid_out = pid_handler_->Tick(aixs_err.pitch, true);
        }
        for (int i = 0; i < 3; i++)
        {
            cmd_data_.dir_steer_state[i] = stateIn;
            if (singleSideFixed == true)
            {
                if (i == 2)
                    v_aix = tar_v_aix_ + pid_out; // 如果是主动驱动轮，误差为正时应该增大轴向速度
                else
                    v_aix = tar_v_aix_ - pid_out; // 辅助驱动轮，误差为正时应该减小轴向速度
            }
            float vel_total = sqrt(v_aix * v_aix + v_cir * v_cir); // 速度的数值大小
            float vel_dir = atan2(v_aix, v_cir);                   // 速度的方向 , 这里的角度范围为[-PI , PI]
            // 将角度范围调整到（0,PI]，并为此修正速度大小
            if (vel_dir <= 0.0f)
            {
                vel_dir += 1.0f * PI;
                vel_total = -vel_total;
            }
            cmd_data_.dir_steer_dir[i] = vel_dir;   // 舵轮舵向的角度
            cmd_data_.dir_steer_vel[i] = vel_total; // 舵轮舵向的速度
            // std::cout << "Wheel ID: " << i
            //           << " Dir: " << cmd_data_.dir_steer_dir[i]
            //           << " Vel: " << cmd_data_.dir_steer_vel[i]
            //           << std::endl;
        }
    }
}

void SINGLE_SIDE_CTRL::fix_quat(){
    if(imu_handler_ != nullptr){
        imu_handler_->fix_quat();
        singleSideFixed = true;  // 设置单侧固定状态为true
    }
}

void SINGLE_SIDE_CTRL::release_quat(){
    singleSideFixed = false;  // 释放IMU的四元数，恢复到正常状态
}


// ! ========================== push ctrl ===========================
// ! ========================== push ctrl ===========================
PUSH_CTRL::PUSH_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh){
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
    }else{
        nh_ = nh;
    }
    cmd_pub_ = nh_->advertise<PUSH_CMD_TYPE>(cmd_topic, 1);
    val_sub_ = nh_->subscribe(val_topic, 1, &PUSH_CTRL::val_callback, this);
    std::cout<< GREEN_STRING << BLOD_STRING << "push ctrl node start\n" 
            << " cmd_topic: " << cmd_topic << "\n"
            << " val_topic: " << val_topic
            << RESET_STRING << std::endl;
    
    // 初始化控制指令
    cmd_data_.tar_length_f = 20.0f;  // 前推杆的目标长度
    cmd_data_.tar_length_b = 20.0f;  // 后推杆的目标长度
    cmd_data_.tar_length_m = 15.0f;  // 中推杆的目标长度
}

PUSH_CTRL::~PUSH_CTRL(){
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}

void PUSH_CTRL::pub_cmd(){
    cmd_pub_.publish(cmd_data_);
}

void PUSH_CTRL::set_cmd(float tar_length_f , float tar_length_b , float tar_length_m){
    cmd_data_.tar_length_f = tar_length_f;
    cmd_data_.tar_length_b = tar_length_b;
    cmd_data_.tar_length_m = tar_length_m;
}

void PUSH_CTRL::val_callback(const PUSH_VAL_CPTR &msg){
    if(msg != nullptr)
    val_data_ = *msg;
}



