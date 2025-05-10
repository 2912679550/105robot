#include "ros/ros.h"
#include "ros_topic_channel.hpp"
#include "robot_ctrl/single_side_cmd.h"
#include "robot_ctrl/single_side_val.h"
#include "robot_ctrl/tcp_motion_cmd.h"
#include "robot_ctrl/push_board_cmd.h"
#include "robot_ctrl/push_board_val.h"

#define STEER_WHEEL 0   // 舵轮轮电机数组序号
#define STEER_DIR 1     // 舵轮方向电机数组序号
#define MECH_MOTOR 2    // 机构电机数组序号

typedef enum    // 与底层32对应，舵轮当前的工作状态
{
    STOP,           // 0 停止
    NORMAL,     // 1 正常
    TORQUE,     // 2 扭矩
    RESET,      // 3 复位
    RESET_OVER  // 4 完成复位
} steerState;



class SINGLE_SIDE_CTRL
{
public:
    // 构造与析构函数 
    SINGLE_SIDE_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh = nullptr);
    ~SINGLE_SIDE_CTRL();
    // 数据存储，用于存放32段发回来的一些运行数据，可以通过外部访问，辅助主程序逻辑
    STM_ROBOT_VAL_TYPE val_data_;
    ROBOT_STM_CMD_TYPE cmd_data_;

    // 外部接口，用于向32发布控制指令
    void pub_cmd();
    void set_tight(bool tightFlag);
    void set_angle(float angle);
    void set_steer(steerState stateIn , float v_aix = 0.0f , float v_cir = 0.0f);

private:
    ros::NodeHandle *nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber val_sub_;
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

class MAIN_ROBOT
{
public:
    MAIN_ROBOT(ros::NodeHandle* nh_ = nullptr);
    ~MAIN_ROBOT();
    

    SINGLE_SIDE_CTRL* front_side_;
    SINGLE_SIDE_CTRL* back_side_;
    PUSH_CTRL* push_ctrl_;

    void pubCmd(){
        front_side_->pub_cmd();
        back_side_->pub_cmd();
        push_ctrl_->pub_cmd();
    };

private:
    ros::NodeHandle* nh_;
    ros::Publisher tcp_pub_;
    ros::Subscriber tcp_sub_;
    void motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg);
};