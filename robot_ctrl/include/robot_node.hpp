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

    void robot_ctrl(bool printFlag = false);
    // 用于在外部通过键盘按键直接调用cmd_callback并进行人为回调处理
    void cmd_hand_maked(TCP_ROBOT_CMD_TYPE* msg);

    
private:
    ros::NodeHandle* nh_;
    ros::Publisher tcp_pub_;
    ros::Subscriber tcp_sub_;
    void motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg);

    void pubCmd();
};