#include "robot_params.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "ros/ros.h"     

int TS;

float odomValueCoeff_f[3]; //  = {1.040268, 1.05457, 1.016019};
float odomValueCoeff_b[3]; //  = {1.048881, 1.063183, 0.994876}; // 里程计值的缩放系数

float steerVelRange[2] ; // 舵轮电机的速度范围，单位为m/s

// todo IMU控制PID的参数
// * IMU俯仰角控制
float imu_pitch_P[2];
float imu_pitch_I[2]; // 前后
float imu_pitch_D[2]; // 前后侧IMU俯仰角PID参数
float imu_pitch_Iband[2] ; // 前后侧IMU俯仰角PID积分死区
float imu_pitch_accIRange[2] ; // 前后侧IMU俯仰角PID积分限幅范围
float imu_pitch_outRange[2];
// * IMU偏航角控制
float imu_yaw_P[2];
float imu_yaw_I[2]; // 前后
float imu_yaw_D[2]; // 前后侧IMU俯仰角PID参数
float imu_yaw_Iband[2]; // 前后侧IMU俯仰角PID积分死区
float imu_yaw_accIRange[2]; // 前后侧IMU俯仰角PID积分限幅范围
float imu_yaw_outRange[2];

// todo 弯折角控制
float body_angle_range[2];

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle & n){
    std::string config_file;
    
    // 尝试从私有命名空间获取参数
    ros::NodeHandle nh_private("~");
    if (!nh_private.getParam("config_file", config_file)) {
        // 如果私有命名空间没有，尝试从全局命名空间获取
        if (!n.getParam("config_file", config_file)) {
            ROS_ERROR("Failed to load config_file parameter from both private and global namespace");
            ROS_ERROR("Make sure to set the config_file parameter in your launch file");
            return;
        }
    }
    
    ROS_INFO_STREAM("Config file path: " << config_file);
    
    // 检查文件是否存在
    std::ifstream file_check(config_file);
    if (!file_check.good()) {
        ROS_ERROR_STREAM("Config file does not exist: " << config_file);
        return;
    }
    file_check.close();
    
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR_STREAM("ERROR: Could not open config file: " << config_file);
        return;
    }

    // 读取系统参数
    TS = (int)fsSettings["TS"];
    
    // 读取里程计系数
    cv::FileNode odom_coeff_node = fsSettings["odom_coeff"];
    cv::FileNode front_node = odom_coeff_node["front"];
    cv::FileNode back_node = odom_coeff_node["back"];
    
    for(int i = 0; i < 3; i++){
        odomValueCoeff_f[i] = (float)front_node[i];
        odomValueCoeff_b[i] = (float)back_node[i];
    }
    
    // 读取舵轮速度范围
    cv::FileNode steer_vel_node = fsSettings["steer_vel_range"];
    steerVelRange[0] = (float)steer_vel_node[0];
    steerVelRange[1] = (float)steer_vel_node[1];
    
    // 读取IMU俯仰角PID参数
    cv::FileNode imu_pitch_node = fsSettings["imu_pitch"];
    cv::FileNode pitch_P = imu_pitch_node["P"];
    cv::FileNode pitch_I = imu_pitch_node["I"];
    cv::FileNode pitch_D = imu_pitch_node["D"];
    cv::FileNode pitch_Iband = imu_pitch_node["Iband"];
    cv::FileNode pitch_acc_I = imu_pitch_node["acc_I_range"];
    cv::FileNode pitch_out = imu_pitch_node["out_range"];
    
    for(int i = 0; i < 2; i++){
        imu_pitch_P[i] = (float)pitch_P[i];
        imu_pitch_I[i] = (float)pitch_I[i];
        imu_pitch_D[i] = (float)pitch_D[i];
        imu_pitch_Iband[i] = (float)pitch_Iband[i];
    }
    imu_pitch_accIRange[0] = (float)pitch_acc_I[0];
    imu_pitch_accIRange[1] = (float)pitch_acc_I[1];
    imu_pitch_outRange[0] = (float)pitch_out[0];
    imu_pitch_outRange[1] = (float)pitch_out[1];
    
    // 读取IMU偏航角PID参数
    cv::FileNode imu_yaw_node = fsSettings["imu_yaw"];
    cv::FileNode yaw_P = imu_yaw_node["P"];
    cv::FileNode yaw_I = imu_yaw_node["I"];
    cv::FileNode yaw_D = imu_yaw_node["D"];
    cv::FileNode yaw_Iband = imu_yaw_node["Iband"];
    cv::FileNode yaw_acc_I = imu_yaw_node["acc_I_range"];
    cv::FileNode yaw_out = imu_yaw_node["out_range"];
    
    for(int i = 0; i < 2; i++){
        imu_yaw_P[i] = (float)yaw_P[i];
        imu_yaw_I[i] = (float)yaw_I[i];
        imu_yaw_D[i] = (float)yaw_D[i];
        imu_yaw_Iband[i] = (float)yaw_Iband[i];
    }
    imu_yaw_accIRange[0] = (float)yaw_acc_I[0];
    imu_yaw_accIRange[1] = (float)yaw_acc_I[1];
    imu_yaw_outRange[0] = (float)yaw_out[0];
    imu_yaw_outRange[1] = (float)yaw_out[1];
    
    // 读取弯折角控制参数
    cv::FileNode body_angle_node = fsSettings["body_angle"];
    cv::FileNode angle_range = body_angle_node["range"];
    body_angle_range[0] = (float)angle_range[0];
    body_angle_range[1] = (float)angle_range[1];
    
    fsSettings.release();
    
    // 打印读取的参数用于调试
    std::cout   << "Parameters loaded successfully:" << std::endl;
    std::cout   << "TS: " << TS << std::endl;
    std::cout   << "Front odom coeff: [" << odomValueCoeff_f[0] << ", "
                << odomValueCoeff_f[1] << ", " << odomValueCoeff_f[2] << "]" << std::endl;
    std::cout   << "Back odom coeff: [" << odomValueCoeff_b[0] << ", "
                << odomValueCoeff_b[1] << ", " << odomValueCoeff_b[2] << "]" << std::endl;
    std::cout   << "Steer Vel Range: [" << steerVelRange[0] << ", "
                << steerVelRange[1] << "]" << std::endl;
    std::cout   << "Body Angle Range: [" << body_angle_range[0] << ", "
                << body_angle_range[1] << "]" << std::endl;
    std::cout   << "IMU Pitch PID: P[" << imu_pitch_P[0] << ", " << imu_pitch_P[1]
                << "], I[" << imu_pitch_I[0] << ", " << imu_pitch_I[1]
                << "], D[" << imu_pitch_D[0] << ", " << imu_pitch_D[1]
                << "], Iband[" << imu_pitch_Iband[0] << ", " << imu_pitch_Iband[1]
                << "], accIRange[" << imu_pitch_accIRange[0] << ", " << imu_pitch_accIRange[1]
                << "], outRange[" << imu_pitch_outRange[0] << ", " << imu_pitch_outRange[1] << "]" << std::endl;
    std::cout   << "IMU Yaw PID: P[" << imu_yaw_P[0] << ", " << imu_yaw_P[1]
                << "], I[" << imu_yaw_I[0] << ", " << imu_yaw_I[1]
                << "], D[" << imu_yaw_D[0] << ", " << imu_yaw_D[1]
                << "], Iband[" << imu_yaw_Iband[0] << ", " << imu_yaw_Iband[1]
                << "], accIRange[" << imu_yaw_accIRange[0] << ", " << imu_yaw_accIRange[1]
                << "], outRange[" << imu_yaw_outRange[0] << ", " << imu_yaw_outRange[1] << "]" << std::endl;
    std::cout   << "Body Angle Range: [" << body_angle_range[0] << ", "
                << body_angle_range[1] << "]" << std::endl;
}

// extern float steerVelRange[2] = {
//     -0.2f,
//     0.2f
// }; // 舵轮电机的速度范围，单位为m/s

// // todo IMU控制PID的参数
// // * IMU俯仰角控制
// extern float imu_pitch_P[2] = {
//     0.008f,
//     0.008f};
// extern float imu_pitch_I[2] = {
//     0.0001f,
//     0.0001f}; // 前后
// extern float imu_pitch_D[2] = {
//     0.0001f,
//     0.0001f}; // 前后侧IMU俯仰角PID参数
// extern float imu_pitch_Iband[2] = {
//     5.f,
//     5.f}; // 前后侧IMU俯仰角PID积分死区
// extern float imu_pitch_accIRange[2] = {
//     -0.1f,
//     0.1f}; // 前后侧IMU俯仰角PID积分限幅范围
// extern float imu_pitch_outRange[2] = {
//     -0.1f,
//     0.1f};
// // * IMU偏航角控制
// extern float imu_yaw_P[2] = {
//     0.008f,
//     0.008f};
// extern float imu_yaw_I[2] = {
//     0.0001f,
//     0.0001f}; // 前后
// extern float imu_yaw_D[2] = {
//     0.0001f,
//     0.0001f}; // 前后侧IMU俯仰角PID参数
// extern float imu_yaw_Iband[2] = {
//     10.f,
//     10.f}; // 前后侧IMU俯仰角PID积分死区
// extern float imu_yaw_accIRange[2] = {
//     -0.1f,
//     0.1f}; // 前后侧IMU俯仰角PID积分限幅范围
// extern float imu_yaw_outRange[2] = {
//     -0.1f,
//     0.1f};

// // todo 弯折角控制
// extern float body_angle_range[2] = {
//     -20.0f,
//     90.0f
// };
