#ifndef TIMER_
#define TIMER_

#include <time.h>
#include <iostream>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

using namespace std;

// 自定义定时器类，如果输入double类型，单位为ms的时间参数，
// // 则定时器会自动从 0～该时间 进行循环
// 如果输入值为-1，则定时器为开环计时状态，输出值为reset函数执行后到调用时的时间间隔，
//  // 需要在定时器外部设置循环等逻辑来reset定时器
class MYTIMER{
    public:
        MYTIMER(/* double input */);
        double ResetTime = -1; //reset默认不使用
        void reset();

        double get_ms();
        double get_sec();
        double get_us();
        double get_ms_duration(bool print = false); 
        // void pause();
        void delay_ms(float DelayTime);
        void delay_us(float DelayTime);
    private:
        double duration_last = 0;
        double duration_now = 0;
        double CurrentTime_ms = 0;
        double CurrentTime_sec = 0;
        double CurrentTime_us = 0;
        timespec Start;
        timespec End;
};



#endif