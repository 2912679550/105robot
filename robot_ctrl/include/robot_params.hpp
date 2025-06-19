#ifndef ROBOT_PARAMS_HPP
#define ROBOT_PARAMS_HPP
// 用于存储一些机器人的结构参数与控制配置等，大部分和32段的参数一致
const float PI = 3.14159265358979323846f; // 圆周率

const  float mechAngleRange[2] = {
    61.0f,
    125.0f
}; // 两臂夹角传感器的0点值被定义为模型中直观感觉的补角，单位为度，在PC端转换为认知角度

// y = 0.0082x6 - 0.2654x5 + 3.3116x4 - 20.016x3 + 61.627x2 - 83.399x + 67.07
// y = 0.0082x6 - 0.2654x5 + 3.3116x4 - 20.016x3 + 61.627x2 - 83.399x + 67.07
// y = -0.1888x3 + 3.458x2 - 7.3305x + 33.593
// const float dia2mechAngelCoeff[4] = {
    //     -0.1888f,
    //     3.458f,
    //     -7.3305f,
    //     33.593f
    // };
    
// matlab:   -0.5344    4.4747  -12.0389    8.7028   16.6731  -14.2722   25.3489
const float dia2mechAngelCoeff[7] = {
    -0.5344f,
    4.4747f,
    -12.0389f,
    8.7028f,
    16.6731f,
    -14.2722f,
    25.3489f
}; // 直径到夹角的六维多项式系数,来自excel拟合

extern float odomValueCoeff_f[3] ; // = {1.040268, 1.05457, 1.016019};
extern float odomValueCoeff_b[3] ; // = {1.048881, 1.063183, 0.994876}; // 里程计值的缩放系数

// const float dia2mechAngelCoeff[7] = {
//     0.0082f,
//     -0.2654f,
//     3.3116f,
//     -20.016f,
//     61.627f,
//     -83.399f,
//     67.07f
// }; // 直径到夹角的六维多项式系数,来自excel拟合


const float steerVelRange[2] = {
    -0.2f,
    0.2f
}; // 舵轮电机的速度范围，单位为m/s

const float imu_pitch_P[2] = {
    0.008f,
    0.008f};

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



