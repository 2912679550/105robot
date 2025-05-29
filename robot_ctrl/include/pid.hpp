#include "math.h"
#include "ros_topic_channel.hpp"

typedef struct{
    float p, i, d;
    float n = 1.0f;
    float ts = 0.005f; // 对应主程序控制周期                             // PID参数 分别为：比例、积分、微分、采样周期
    float outIMin, outIMax, Iband;  // 积分限幅、积分死区
    float outMin, outMax;           // 输出限幅
} PID_PARAM;


class Pid
{
private:
float accI, accD, accIMax, accIMin, accDMax, accDMin, outMax, outMin, Iband;
public:
    float p, i, d, n, ts;
    Pid(float p, float i, float d, float n, float ts,  float outIMin, float outIMax, float Iband, float outMin, float outMax);
    Pid(PID_PARAM *param);
    float Tick(float diff , bool  printFlag = false);
    void Reset();
    void SetParam(float p, float i);
};