#include "ros/ros.h"
#include "robot_ctrl/single_side_cmd.h"
#include "robot_ctrl/single_side_val.h"
#include "robot_ctrl/tcp_motion_cmd.h"
#include "robot_ctrl/push_board_cmd.h"
#include "robot_ctrl/push_board_val.h"
#include "imu_handler.hpp"
#include "pid.hpp"
#include "ros_topic_channel.hpp"
#include "robot_params.hpp"
#include <eigen3/Eigen/Dense>
#include "Timer.hpp"


typedef enum    // 与底层32对应，舵轮当前的工作状态
{
    STOP,           // 0 停止
    NORMAL,     // 1 正常
    TORQUE,     // 2 扭矩
    RESET,      // 3 复位
    RESET_OVER  // 4 完成复位
} steerState;

static float TIGHT_LENGTH_LIMIT[2] = {47.0f , 58.0f};  // 夹紧长度范围

class MICRO_ODOM{
public:
    MICRO_ODOM();
    ~MICRO_ODOM();
    // 轴向和周向的里程计数据
    float odom_axis = 0.0f;  // 轴向里程计
    float odom_cir = 0.0f;   // 周向里程计
    // 更新里程计数据
    void update(STM_ROBOT_VAL_TYPE* val_data, bool printFlag = false);
    void reset();  // 重置里程计数据
    void start_odom(){startOdom = true;}    // 开始里程计数据处理
    void pause_odom(){startOdom = false;}   // 暂停里程计数据处理
    void set_cur_val(float axis, float cir);  // 设置当前里程计数据(里程计采用增量式，可以在这里直接修改累加的基础值)
private:
    Eigen::Vector2f pre_position_[3];         // 存储接收到的上一次三个轮子的定位数据
    Eigen::Vector2f cur_position_[3];         // 存储接收到的当前三个轮子的定位数据

    bool startOdom = false;  // 是否开始里程计数据处理

    bool resetFlag = true;  // 复位标志位，true表示需要复位里程计数据
};

class SINGLE_SIDE_CTRL
{
public:
    // 构造与析构函数 
    SINGLE_SIDE_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh = nullptr);
    ~SINGLE_SIDE_CTRL();
    // 数据存储，用于存放32段发回来的一些运行数据，可以通过外部访问，辅助主程序逻辑
    STM_ROBOT_VAL_TYPE val_data_;
    ROBOT_STM_CMD_TYPE cmd_data_;
    // 功能类
    IMU_HANDLER* imu_handler_;  // IMU数据处理类
    Pid* pid_handler_;          // PID控制器，用于姿态控制
    MICRO_ODOM* odom_handler_;  // 里程计处理类，用于处理轴向和周向的里程计数据
    MYTIMER* tight_timer_;  // 定时器处理类，用于定时发布控制指令

    bool tarTightFlag_ = false;  // 目标夹紧状态
    bool pre_tightFlag_ = false;  // 上一次夹紧状态
    bool cur_tightFlag_ = false;  // 当前夹紧状态
    
    void single_side_ctrl();  // 单侧控制逻辑
    // 外部接口，用于向32发布控制指令
    void pub_cmd();
    void set_tight(bool tightFlag);
    void set_tight(float length);   // 用于直接配置期望压缩弹簧的长度
    void set_angle(float angle);
    void set_steer(steerState stateIn , float v_aix = 0.0f , float v_cir = 0.0f);

    void create_imu(std::string imu_topic , int imu_id);
    void fix_quat();
    void release_quat();  // 释放IMU的四元数，恢复到正常状态

private:
    ros::NodeHandle *nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber val_sub_;
    ros::Subscriber imu_sub_;

    bool singleSideFixed = false;

    int imu_id_;  // IMU的ID，用于区分前侧和后侧IMU
    steerState steer_state_ = steerState::STOP;  // 当前舵轮的工作状态
    float tar_v_aix_ = 0.0f;  // 目标轴向速度
    float tar_v_cir_ = 0.0f;  // 目标周向速度

    void val_callback(const STM_ROBOT_VAL_CPTR &msg);
};

class PUSH_CTRL
{
public:
    PUSH_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh = nullptr);
    ~PUSH_CTRL();
    // 存储数据
    PUSH_VAL_TYPE val_data_;
    PUSH_CMD_TYPE cmd_data_;
    // 外部接口
    void pub_cmd();
    void set_cmd(float tar_length_f , float tar_length_b , float tar_length_m);
private:
    ros::NodeHandle *nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber val_sub_;
    void val_callback(const PUSH_VAL_CPTR &msg);
};


