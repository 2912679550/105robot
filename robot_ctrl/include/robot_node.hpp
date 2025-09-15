#pragma once
#include "ros/ros.h"
#include "ros_topic_channel.hpp"
#include "board_ctrl.hpp"
#include "pipe_solver/pipeControler.hpp"

#define STEER_WHEEL 0   // 舵轮轮电机数组序号
#define STEER_DIR 1     // 舵轮方向电机数组序号
#define MECH_MOTOR 2    // 机构电机数

enum ROBOT_STATE{
    MOTION_STOP,
    NORMAL_HAND_CTRL,   // 正常手操遥控
    PLAN_MOTION,        // 规划器运动状态
    IN_PIPE,       // 自动进弯
    OUT_PIPE,      // 自动出弯
};

class POSE_CLOSED_LOOP{
public: 
    POSE_CLOSED_LOOP(std::string front_imu_topic, std::string back_imu_topic,
                    SINGLE_SIDE_CTRL* front_handle , 
                    SINGLE_SIDE_CTRL* back_handle ,  ros::NodeHandle* nh = nullptr);
    ~POSE_CLOSED_LOOP();

    IMU_HANDLER* front_imu_handler_;
    IMU_HANDLER* back_imu_handler_;
    // * 存储PID输出
    float pid_out_p_ = 0.0f; // 俯仰角PID输出
    float pid_out_y_ = 0.0f; // 偏航角PID

    /**
     * @brief 开启机器人姿态闭环控制，并在内部存储前后IMU的姿态数据
     * 
     */
    void turn_on_close_loop_(); 
    /**
     * @brief 关闭机器人姿态闭环控制
     * 
     */
    void turn_off_close_loop_(); 
    /**
     * @brief 执行姿态闭环PID计算，并将结果存储在pid_out_p_和pid_out_y_中
     * 
     * @param printFlag 是否打印调试信息
     */
    void close_loop_pid_(bool printFlag = false);
    void set_pitch_adjust(float adjust_pitch);
    float get_pitch_err();

private:
    ros::NodeHandle* nh_;
    SINGLE_SIDE_CTRL* front_handle_;
    SINGLE_SIDE_CTRL* back_handle_;
    // IMU数据处理类
    Pid* pid_pitch_;  // 俯仰角PID控制器
    Pid* pid_yaw_;    // 偏航角PID控制器

    bool enable_close_loop_ = false; // 是否开启姿态闭环
    float pitch_adj_ = 0.0f; // 期望俯仰角微调值
};

class MOTION_PLAN{
public:
    // 支持外部配置的集中运动规划模式
    enum MOTION_MODE{
        IDLE = 0,
        STEP = 1,           // 步进运动
        SCAN = 2,           // 扫查运动
        FULL_PIPE = 3,      // 管道全覆盖运动（弓字形轨迹）
    };
    float default_speed_ = 0.03f;  // 默认运动速度
    
    void set_motion_range(
        float cur_aix , float cur_cir,
        float _step_axis, float _step_cir , int mode);   // 设置步进或扫查运动的范围
    steerState motion_plan_(float* result_aix , float* result_cir, 
        float cur_aix , float cur_cir,
        bool printFlag = false); // 运动规划主函数，输出轴向与周向速度
    void reset_motion(); // 重置运动规划器

private:
    // 用于在轴向与周向步进或扫查时，存储机器人的运动范围信息
    std::pair<Eigen::Vector2f, Eigen::Vector2f> motion_range;   
    int mode_ = MOTION_MODE::IDLE;      // 当前运动模式
    float trace_distance_ = 0.0f;  // 起点与终点的距离
    float dir_ = 0.0f; // 运动方向
    bool motion_en_ = false; // 运动使能
    bool scan_positive_en_ = true; // 扫查正向使能
    
    // * 用于进行管道弓字形扫查的标志位
    // bool pipe_scan_en_ = false;
    // bool pipe_scan_is_aix_ = true;
    int full_pipe_sub_id = 0;
    std::pair<Eigen::Vector2f, Eigen::Vector2f> *motion_range_cpy;
    std::vector< std::pair<Eigen::Vector2f, Eigen::Vector2f> > motion_range_sub;
};

class MAIN_ROBOT
{
public:
    MAIN_ROBOT(ros::NodeHandle* nh_ = nullptr);
    ~MAIN_ROBOT();
    // * 内置控制器
    SINGLE_SIDE_CTRL* front_side_;
    SINGLE_SIDE_CTRL* back_side_;
    PUSH_CTRL* push_ctrl_;
    PipeController* pipe_controller_ = nullptr; // 管道控制器
    POSE_CLOSED_LOOP* pose_closed_ctrl_ = nullptr; // 姿态闭环控制器
    MOTION_PLAN* motion_planner_ = nullptr; // 运动规划器

    float robot_axis_odom_ = 0.0f;  // 机器人轴向里程计数据
    float robot_cir_odom_ = 0.0f;   // 机器人周向里程计数据

    ROBOT_STATE robot_state_ = NORMAL_HAND_CTRL; // 机器人当前的工作状态
    ROBOT_STATE pre_robot_state_ = NORMAL_HAND_CTRL; // 机器人上一次的工作状态
    int cur_state_repeat_count_ = 0; // 当前状态被重复按下的次数

    void robot_ctrl(bool printFlag = false);
    // 用于在外部通过键盘按键直接调用cmd_callback并进行人为回调处理
    void cmd_hand_maked(TCP_ROBOT_CMD_TYPE* msg);
    void change_state(ROBOT_STATE new_state); // 改变机器人状态
private:
    ros::NodeHandle* nh_;
    ros::Publisher tcp_pub_;
    ros::Subscriber tcp_sub_;
    
    // 当步进运动或扫查运动的标志位使能后，主控制状态机将自动调用motion_range中存储的信息
    // 步进运动只调用second
    // 扫查运动调用first和second
    std::pair<Eigen::Vector2f, Eigen::Vector2f> motion_range;   // 用于在轴向与周向步进或扫查时，存储机器人的运动范围信息
    
    // 存储接收到的期望速度
    float tar_v_aix_ = 0.0f;  // 目标轴向速度
    float tar_v_cir_ = 0.0f;  // 目标周向速度

    bool front_odom_en_ = false;  // 前侧里程计使能
    bool back_odom_en_ = false;   // 后侧里程计使能

    void odom_handler(bool printFlag = false); // 里程计处理函数
    void motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg);
    void pubCmd();

    // * 用于自动进弯控制
    bool pipe_cali_flag_ = false; // 自动进弯标定使能
    float saved_odom_aix_[2] = {0.0f, 0.0f}; // 管道起始位置轴向里程计坐标
    float saved_odom_cir_[2] = {0.0f, 0.0f}; // 管道起始位置周向里程计坐标
    float saved_pipe_r_[2] = {0.0f, 0.0f}; // 管道半径
    tf::Quaternion saved_quat_[2]; // 管道起始位置IMU四元数
    void save_odom_and_imu(); // 保存里程计与IMU数据

};

