#include "ros/ros.h"
#include "ros_topic_channel.hpp"
#include "board_ctrl.hpp"



#define STEER_WHEEL 0   // 舵轮轮电机数组序号
#define STEER_DIR 1     // 舵轮方向电机数组序号
#define MECH_MOTOR 2    // 机构电机数组序号




class MAIN_ROBOT
{
public:
    MAIN_ROBOT(ros::NodeHandle* nh_ = nullptr);
    ~MAIN_ROBOT();
    

    SINGLE_SIDE_CTRL* front_side_;
    SINGLE_SIDE_CTRL* back_side_;
    PUSH_CTRL* push_ctrl_;

    float robot_axis_odom_ = 0.0f;  // 机器人轴向里程计数据
    float robot_cir_odom_ = 0.0f;   // 机器人周向里程计数据

    // 当步进运动或扫查运动的标志位使能后，主控制状态机将自动调用motion_range中存储的信息
    // 步进运动只调用second
    // 扫查运动调用first和second
    std::pair<Eigen::Vector2f, Eigen::Vector2f> motion_range;   // 用于在轴向与周向步进或扫查时，存储机器人的运动范围信息
    void set_motion_range(float _step_axis, float _step_cir);   // 设置步进或扫查运动的范围

    void robot_ctrl(bool printFlag = false);
    // 用于在外部通过键盘按键直接调用cmd_callback并进行人为回调处理
    void cmd_hand_maked(TCP_ROBOT_CMD_TYPE* msg);

    
private:
    ros::NodeHandle* nh_;
    ros::Publisher tcp_pub_;
    ros::Subscriber tcp_sub_;
    void motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg);

    bool front_odom_en_ = false;  // 前侧里程计使能
    bool back_odom_en_ = false;   // 后侧里程计使能
    bool step_moiton_en_ = false; // 步进运动使能
    bool scan_motion_en_ = false; // 扫查运动使能
    bool scan_positive_en_ = true; // 扫查正向使能
    void odom_handler(bool printFlag = false); // 里程计处理函数

    void pubCmd();
};