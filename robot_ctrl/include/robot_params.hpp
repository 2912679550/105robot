// 用于存储一些机器人的结构参数与控制配置等，大部分和32段的参数一致
const float PI = 3.14159265358979323846f; // 圆周率

const float mechAngleRange[2] = {
    61.0f,
    125.0f
}; // 两臂夹角传感器的0点值被定义为模型中直观感觉的补角，单位为度，在PC端转换为认知角度

const float steerVelRange[2] = {
    -0.2f,
    0.2f
}; // 舵轮电机的速度范围，单位为m/s
