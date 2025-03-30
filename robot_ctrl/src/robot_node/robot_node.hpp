#include "ros/ros.h"
#include "ros_topic_channel.hpp"
#include "robot_ctrl/single_side_cmd.h"
#include "robot_ctrl/single_side_val.h"
#include "robot_ctrl/tcp_motion_cmd.h"

#define STEER_WHEEL 0   // 舵轮轮电机数组序号
#define STEER_DIR 1     // 舵轮方向电机数组序号
#define MECH_MOTOR 2    // 机构电机数组序号

class SINGLE_SIDE_CTRL
{
public:
    // 构造与析构函数 
    SINGLE_SIDE_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh = nullptr);
    ~SINGLE_SIDE_CTRL();
    // 数据存储，用于存放32段发回来的一些运行数据，可以通过外部访问，辅助主程序逻辑
    STM_ROBOT_VAL_TYPE val_data_;

    // 外部接口，用于向32发布控制指令
    void pub_cmd(const ROBOT_STM_CMD_TYPE &cmdIn);
private:
    ros::NodeHandle *nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber val_sub_;
    void val_callback(const STM_ROBOT_VAL_CPTR &msg);
};

class MAIN_ROBOT
{
public:
    MAIN_ROBOT(ros::NodeHandle* nh_ = nullptr);
    ~MAIN_ROBOT();

    SINGLE_SIDE_CTRL* front_side_;
    SINGLE_SIDE_CTRL* back_side_;
private:
    ros::NodeHandle* nh_;
    ros::Publisher tcp_pub_;
    ros::Subscriber tcp_sub_;
    void motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg);
};