#include "ros/ros.h"    
#include "ros_topic_channel.hpp"
#include "eigen3/Eigen/Dense"
#include "tf/transform_datatypes.h"

// *  前侧陀螺仪的安装相当于沿着机器人坐标系的 x 轴旋转了 -90 度

// *  后侧陀螺仪的安装相当于先沿着机器人坐标系的 x 轴旋转了 90 度
// *  然后沿着机器人坐标系 y 轴旋转了 180 度

// 定义旋转矩阵，将IMU体坐标系转换为机器人坐标系
static const double IMU_FRONT_ROTATE[3][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0},
    {0.0,-1.0, 0.0},
};  // 前侧IMU坐标系旋转矩阵
static const double IMU_BACK_ROTATE[3][3] = {
    {0.0, 0.0, 1.0},
    {1.0, 0.0, 0.0},
    {0.0, -1.0, 0.0}
};  // 后侧IMU坐标系旋转矩阵

typedef enum {
    FRONT = 0,  // 前侧IMU
    BACK = 1    // 后侧IMU
} IMU_ID;

struct IMU_POSE{
    double roll;   // 滚转角
    double pitch;  // 俯仰角
    double yaw;    // 偏航角
    // float acc_x;  // x轴加速度
    // float acc_y;  // y轴加速度
    // float acc_z;  // z轴加速度
    // float gyro_x; // x轴角速度
    // float gyro_y; // y轴角速度
    // float gyro_z; // z轴角速度
};

class IMU_HANDLER{
public:
    IMU_HANDLER(std::string imu_topic, ros::NodeHandle *nh = nullptr);
    ~IMU_HANDLER();

    // 不同形式的IMU数据
    IMU_POSE ground_truth;
    IMU_POSE pose_cur;
    IMU_POSE pose_pre;  // 上一帧的IMU数据

    // R_b_i  表示 IMU体坐标系到机器人坐标系的旋转矩阵
    tf::Matrix3x3* imu_robot_matrix = nullptr;  // IMU数据转换矩阵，指向对应的旋转矩阵

    bool imu_reset_flag = true;  // IMU复位标志位，true表示需要复位IMU数据

    void reset_pose();

private:
    ros::NodeHandle *nh_;
    ros::Subscriber imu_sub_;
    void imu_callback(const IMU_CPTR &msg);
};
