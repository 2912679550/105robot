#ifndef PUBLIC_HPP
#define PUBLIC_HPP
#include <sys/select.h>
#include <stdio.h>
#include <fstream>


#define PI 3.14159265358979323846

struct POINT_XY{
    float x;
    float y;
};

double LimitProtect(double value, double min, double max);

int MySgn(double value);

char get_char();
// 将角度转换到-pi到pi之间
float angle_stand_pi(float angleIn);
// 将角度转换到0到2pi之间
float angle_stand_2pi(float angleIn);

// 将A坐标系中表示的点旋转变换到B坐标系表示的点中，记从B到A的旋转角度为theta
POINT_XY rotate_trans(POINT_XY point, float theta);
// 将A坐标系中表示的点平移变换到B坐标系表示的点中
POINT_XY linear_trans(POINT_XY point , float x, float y);

// 计算两点之间的距离
float point_distance(POINT_XY point1, POINT_XY point2);

// 根据输入的起点与终点坐标和位姿，按照团队自动生成控制点的方式计算三阶贝塞尔曲线的总长度
float bezier_arc_length(float startPoint[], float endPoint[]);
// 根据输入的起点与终点坐标和位姿，按照团队自动生成控制点的方式计算三阶贝塞尔曲线上的点
POINT_XY bezier_curve_alpha(float t, float startPoint[], float endPoint[]);
// 使用迭代的方式，给定曲线的起点与终点位置与位姿，计算曲线上与目标点最近的点的区间t值
POINT_XY bezier_close_point(POINT_XY interIn,float startPoint[], float endPoint[],POINT_XY targetPoint,int nTimes = 3);
#endif
