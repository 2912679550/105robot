#include "robot_ctrl/tcp_motion_cmd.h"
#include "robot_ctrl/robot_motion_val.h"

// todo ROS话题通道定义与类型转义
#define ROBOT_TCP_CMD "robot_tcp_cmd_topic"             // 由手柄通过TCP发来的控制指令，通过tcp_node广播给机器人控制节点
typedef robot_ctrl::tcp_motion_cmd ROBOT_TCP_CMD_TYPE;  // 机器人控制指令
typedef robot_ctrl::tcp_motion_cmdConstPtr ROBOT_TCP_CMD_CPTR;  // 机器人控制指令指针

#define STM2PC_V_F "stm2pc_v_f"                     // 机器人前侧边在机器人坐标系下的速度
#define STM2PC_V_B "stm2pc_v_b"                     // 机器人后侧边在机器人坐标系下的速度
typedef robot_ctrl::robot_motion_val STM2PC_V_TYPE; // 机器人运动速度
typedef robot_ctrl::robot_motion_valConstPtr STM2PC_V_CPTR; // 机器人运动速度指针



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

