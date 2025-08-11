#pragma once

#include "robot_params.hpp"
#include "autoPipeSolver.hpp"
#include <tf/transform_datatypes.h>

class PipeController {
public:
    PipeController();
    ~PipeController();
    AutoPipeSolver* solver_ = nullptr; // 自动管道求解器
    
    // * 内部状态变量
    bool is_started_ = false; // 是否已经设置了起始位置
    float start_odom_aix_ = 0.0f; // 起始位置轴向里程计坐标
    float start_odom_cir_ = 0.0f; // 起始位置周向里程计坐标
    tf::Quaternion start_imu_quat_; // 起始位置IMU四元数

    
    // * 求解器输入变量
    float pipe_r_ = 360.0 / 2.0; // 管道半径
    float pipe_R_ = 300.0;       // 管道转弯半径


    // * 主要结果变量
    float push_length_ = body_angle_push_baseline; // 推杆长度，单位mm
    float main_wheel_speed_[2] = {0.0f, 0.0f};      // 主动轮轮速，单位m/s, [轴向速度, 周向速度]
    float assist_wheel_speed_[2] = {0.0f, 0.0f};    // 辅助轮轮速，单位m/s, [轴向速度, 周向速度]

    // * 接口函数  
    bool set_start_(float odom_aix , float odom_cir , tf::Quaternion* imu_quat); // 设置起始位置
    bool auto_in_pipe_(float odom_aix , float odom_cir , tf::Quaternion* cur_quat, 
                        float target_v = 0.02f,
                        bool printFlag = false); // 自动进入管道
    bool auto_out_pipe_(float odom_aix , float odom_cir , tf::Quaternion* cur_quat, 
                        float target_v = 0.02f,
                        bool printFlag = false); // 自动退出管道

    bool set_geometry_params_(float R, float r);
private:
};
