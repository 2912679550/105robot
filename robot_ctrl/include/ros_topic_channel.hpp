// ! 核心数据流： 手柄 -> tcp_node ---- [运动指令] ----> 机器人主控程序(cpp) ---- [控制数据] ----> 与32端通信(python)
#include "robot_ctrl/tcp_motion_cmd.h"
#include "robot_ctrl/robot_motion_val.h"
#include "robot_ctrl/single_side_cmd.h"
#include "robot_ctrl/single_side_val.h" 

// todo ROS话题通道定义与类型转义

// * 手柄与PC的收发信道( for tcp_node )
#define TCP_ROBOT_CMD "TCP_ROBOT_CMD_topic"             // 由手柄通过TCP发来的控制指令，通过tcp_node广播给机器人控制节点
typedef robot_ctrl::tcp_motion_cmd TCP_ROBOT_CMD_TYPE;  // 机器人控制指令
typedef robot_ctrl::tcp_motion_cmdConstPtr TCP_ROBOT_CMD_CPTR;  // 机器人控制指令指针
// 由python文件的ether node发出来的32端现在的执行状态，需要通过TCP发给手柄显示UI
#define ROBOT_TCP_VAL "robot_tcp_val_topic"                     // 机器人前侧边在机器人坐标系下的速度
typedef robot_ctrl::robot_motion_val ROBOT_TCP_VAL_TYPE;            // 回传给手柄显示的信息
typedef robot_ctrl::robot_motion_valConstPtr ROBOT_TCP_VAL_CPTR;    // 回传给手柄显示的信息指针

// * 主程序与 ether node 的收发信道( for robot ctrl )
#define ROBOT_STM_CMD_F "robot_stm_cmd_f"             // 前侧的控制指令
#define ROBOT_STM_CMD_B "robot_stm_cmd_b"             // 后侧的控制指令
typedef robot_ctrl::single_side_cmd ROBOT_STM_CMD_TYPE;  // 机器人控制指令
typedef robot_ctrl::single_side_cmdConstPtr ROBOT_STM_CMD_CPTR;  // 机器人控制指令指针

#define STM_ROBOT_VAL_F "stm_robot_f"                     // 机器人前侧发回的当前信息
#define STM_ROBOT_VAL_B "stm_robot_b"                     // 机器人后侧发回的当前信息
typedef robot_ctrl::single_side_val STM_ROBOT_VAL_TYPE;         // 机器人32端当前的运动状态
typedef robot_ctrl::single_side_valConstPtr STM_ROBOT_VAL_CPTR; // 机器人32端当前的运动状态指针



// todo 字符串打印配置宏
// 字体颜色
#define RED_STRING "\033[31m"       //红色
#define GREEN_STRING "\033[32m"     //绿色
#define YELLOW_STRING "\033[33m"    //黄色
#define BLUE_STRING "\033[34m"      //蓝色
#define PURPLE_STRING "\033[35m"    //紫色
#define CYAN_STRING "\033[36m"      //青色
#define WHITE_STRING "\033[37m"     //白色
// 字体格式
#define BLOD_STRING "\033[1m"       //粗体
#define UNDERLINE_STRING "\033[4m"  //下划线
#define BLINK_STRING "\033[5m"      //闪烁
#define REVERSE_STRING "\033[7m"    //反显
#define HIDE_STRING "\033[8m"       //隐藏
#define RESET_STRING "\033[0m"      //重置

