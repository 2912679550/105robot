#ifndef ROBOT_PARAMS_HPP
#define ROBOT_PARAMS_HPP
// 用于存储一些机器人的结构参数与控制配置等，大部分和32段的参数一致
const float PI = 3.14159265358979323846f; // 圆周率

const  float mechAngleRange[2] = {
    61.0f,
    125.0f
}; // 两臂夹角传感器的0点值被定义为模型中直观感觉的补角，单位为度，在PC端转换为认知角度

const float steerVelRange[2] = {
    -0.2f,
    0.2f
}; // 舵轮电机的速度范围，单位为m/s

const float imu_pitch_P[2] = {
    0.004f,
    0.004f};

const float imu_pitch_I[2] = {
    0.0001f,
    0.0001f}; // 前后

const float imu_pitch_D[2] = {
    0.0001f,
    0.0001f}; // 前后侧IMU俯仰角PID参数

const float imu_pitch_Iband[2] = {
    1.5f,
    1.5f}; // 前后侧IMU俯仰角PID积分死区

const float imu_pitch_accIRange[2] = {
    -0.1f,
    0.1f}; // 前后侧IMU俯仰角PID积分限幅范围

const float imu_pitch_outRange[2] = {
    -0.1f,
    0.1f};

#endif



