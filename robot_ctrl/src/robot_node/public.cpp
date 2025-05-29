#include "public.hpp"
#include <math.h>

double LimitProtect(double value, double min, double max){
    if(value > max) return max;
    else if(value < min) return min;
    else return value;
}


int MySgn(double value){
    if(value > 0) return 1;
    else return -1;
}

char get_char()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 5; 
    if (select(1, &rfds, NULL, NULL, &tv) > 0) 
    {
        ch = getchar(); 
    }
    return ch;
}

float angle_stand_pi(float angleIn){
    while(angleIn > PI) angleIn -= 2 * PI;
    while(angleIn < -PI) angleIn += 2 * PI;
    return angleIn;
}

float angle_stand_2pi(float angleIn){
    while(angleIn > 2 * PI) angleIn -= 2 * PI;
    while(angleIn < 0) angleIn += 2 * PI;
    return angleIn;
}

POINT_XY rotate_trans(POINT_XY point, float theta){
    POINT_XY newPoint;
    newPoint.x = point.x * cos(theta) - point.y * sin(theta);
    newPoint.y = point.x * sin(theta) + point.y * cos(theta);
    return newPoint;
}

POINT_XY linear_trans(POINT_XY point , float x, float y){
    POINT_XY newPoint;
    newPoint.x = point.x + x;
    newPoint.y = point.y + y;
    return newPoint;
}

float point_distance(POINT_XY point1, POINT_XY point2){
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

float bezier_arc_length(float startPoint[], float endPoint[]){

    static int SUB_NUM = 15;

    float length = 0.0;
    float t = 0.0;
    for (int i = 0; i < SUB_NUM; i++){
        // 起始点
        t = float(i * 1.0 / SUB_NUM);
        POINT_XY point1 = bezier_curve_alpha(t, startPoint, endPoint);
        // 终点
        t = (i + 1) * 1.0 / SUB_NUM;
        POINT_XY point2 = bezier_curve_alpha(t, startPoint, endPoint);
        // 计算距离
        length += point_distance(point1, point2);
    }
    return length;
}

POINT_XY bezier_curve_alpha(float t, float startPoint[], float endPoint[]){
    static float CTRL_U = 0.83;

    // 自动生成控制点
    float disStart2End = point_distance({startPoint[0], startPoint[1]}, {endPoint[0], endPoint[1]});
    POINT_XY ctrlPoint1;
    POINT_XY ctrlPoint2;
    ctrlPoint1.x = startPoint[0] + CTRL_U * disStart2End * cos(startPoint[2]);
    ctrlPoint1.y = startPoint[1] + CTRL_U * disStart2End * sin(startPoint[2]);
    ctrlPoint2.x = endPoint[0] - CTRL_U * disStart2End * cos(endPoint[2]);
    ctrlPoint2.y = endPoint[1] - CTRL_U * disStart2End * sin(endPoint[2]);
    // 计算贝塞尔曲线上的点
    POINT_XY point;
    point.x = pow(1 - t, 3) * startPoint[0] + 3 * t * pow(1 - t, 2) * ctrlPoint1.x + 3 * pow(t, 2) * (1 - t) * ctrlPoint2.x + pow(t, 3) * endPoint[0];
    point.y = pow(1 - t, 3) * startPoint[1] + 3 * t * pow(1 - t, 2) * ctrlPoint1.y + 3 * pow(t, 2) * (1 - t) * ctrlPoint2.y + pow(t, 3) * endPoint[1];
    return point;
}

// POINT_XY 的x和y分别存储区间的起点和终点
POINT_XY bezier_close_point(POINT_XY interIn,float startPoint[], float endPoint[],POINT_XY targetPoint,int nTimes){
    static int N = 15; // 将输入区间划分为N份

    float minDistance = 100000.0;
    int  minDisIndex = 0;
    for (int i = 0; i <= N;i++){
        float t = float(i * 1.0 / N) * (interIn.y - interIn.x) + interIn.x;
        POINT_XY point = bezier_curve_alpha(t, startPoint, endPoint);
        float distance = point_distance(point, targetPoint);
        if(distance < minDistance){
            minDistance = distance;
            minDisIndex = i;
        }
    }

    // 此时已经得到了最近点的t值与最小距禋
    //  生成新的区间
    POINT_XY newInter;
    if (minDisIndex == 0){
        newInter.x = 0;
        newInter.y = 1.0 / N;
    }else if(minDisIndex == N){
        newInter.x = 1.0 - 1.0 / N;
        newInter.y = 1.0;
    }else{
        newInter.x = float(minDisIndex - 1) / N;
        newInter.y = float(minDisIndex + 1) / N;
    }

    // 递归计算
    if(nTimes > 1){
        nTimes = nTimes - 1;
        return bezier_close_point(newInter, startPoint, endPoint, targetPoint, nTimes - 1);
    }else{
        return newInter;
    }
}
