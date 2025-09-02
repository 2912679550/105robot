#include "Timer.hpp"

using namespace std;

MYTIMER::MYTIMER(){
    reset();
}

void MYTIMER::reset(){
    clock_gettime(CLOCK_MONOTONIC,&Start);
}

double MYTIMER::get_ms(){
    clock_gettime(CLOCK_MONOTONIC,&End);
    CurrentTime_ms = (End.tv_sec - Start.tv_sec) * 1000 + (End.tv_nsec - Start.tv_nsec)/1e6;
    if (ResetTime > 0){
        // 只有在设置使用了计时循环时才进行判断
        if(CurrentTime_ms >= ResetTime){
            // 达到或者超过了重置时间，刷新Start与End
            clock_gettime(CLOCK_MONOTONIC,&Start);
            clock_gettime(CLOCK_MONOTONIC,&End);
            CurrentTime_ms = 0;
        }
    }
    return CurrentTime_ms;
}

double MYTIMER::get_sec(){
    clock_gettime(CLOCK_MONOTONIC,&End);
    CurrentTime_sec = (End.tv_sec - Start.tv_sec) * 1+ (End.tv_nsec - Start.tv_nsec)/1e9;
    if (ResetTime > 0){
        // 只有在设置使用了计时循环时才进行判断
        if(CurrentTime_sec >= ResetTime / 1000){
            // 达到或者超过了重置时间，刷新Start与End
            clock_gettime(CLOCK_MONOTONIC,&Start);
            clock_gettime(CLOCK_MONOTONIC,&End);
            CurrentTime_sec = 0;
        }
    }
    return CurrentTime_sec;
}

double MYTIMER::get_us(){
    clock_gettime(CLOCK_MONOTONIC,&End);
    CurrentTime_us = (End.tv_sec - Start.tv_sec) * 1000 * 1000 + (End.tv_nsec - Start.tv_nsec)/1e3;
    if (ResetTime > 0){
        // 只有在设置使用了计时循环时才进行判断
        if(CurrentTime_us >= ResetTime * 1000){
            // 达到或者超过了重置时间，刷新Start与End
            clock_gettime(CLOCK_MONOTONIC,&Start);
            clock_gettime(CLOCK_MONOTONIC,&End);
            CurrentTime_us = 0;
        }
    }
    return CurrentTime_us;
}

void MYTIMER::delay_ms(float DelayTime){
    double DelayStart = get_ms(); // 记录延时开始的时间
    // 设置了计时循环并且延时开始时剩下的时间中Current时间会重置一次
    if (ResetTime - DelayStart <= DelayTime && ResetTime > 0){
        double TargetTime = DelayTime - (ResetTime - DelayStart);
        while(!(CurrentTime_ms >= TargetTime && CurrentTime_ms <= DelayTime)){
            // ros::spinOnce();
            get_ms();
        }
        return ;
    }else{ // 不会重置
        double TargetTime = DelayStart + DelayTime;
        while(CurrentTime_ms <= TargetTime){
            // ros::spinOnce();
            get_ms();
        }
        return;
    } 
    cout<<"Error logic"<<endl;
    return;
}

void MYTIMER::delay_us(float DelayTime){
    float DelayStart = get_us();
    while(CurrentTime_us - DelayStart <= DelayTime){
        get_us();
    }
}

// 得到两次调用函数之间的时间差，单位为ms
double MYTIMER::get_ms_duration(bool print){
    duration_last = duration_now;
    duration_now = get_ms();
    double duration = duration_now - duration_last;
    if(duration <= 0 && ResetTime>0){
        // 如果使用了时间刷新且中间出现了时间刷新
        duration = ResetTime - duration_last + duration_now;
        // return duration;
    }
    if(print){
        std::cout<<"Duration time between last use function(ms):  "<<duration<<std::endl;
    }
    return duration;
}

