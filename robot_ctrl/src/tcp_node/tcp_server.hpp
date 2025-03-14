#include "pthread.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>

#include <stdio.h>
#include <string>
#include <cstring>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros_topic_channel.hpp"

//Tcp服务器端口号
const int SERVER_PORT = 9527;

// ! TCP功能函数
/*!
 * \brief 字符串分割函数
 * 
 * \param return_data_vec 分割后的vector
 * \param str 传入待分割的字符串
 * \param pattern 分割字符
 */
void buf_split(std::vector<std::string> &return_data_vec, const std::string &str, const std::string &pattern);
void* MessageSubCallback(void* arg);
//指令发布定时器
//子线程调用函数（发布运动指令）
void* InstructionPubCallback(void* arg);

// ! 定义ROS中的消息回调函数
// void PoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);
// void lvbanPoseDataCallback(const geometry_msgs::PoseStampedConstPtr &msg);
// void FanDataCallback(const std_msgs::StringConstPtr& msg);
// void FanDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
// void MotorlineDataCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
// void TrajCallback(const std_msgs::Float32MultiArrayConstPtr& msg);





