#include "ros/ros.h"
#include <signal.h>
#include "robot_ctrl/tcp_motion_cmd.h"
#include "robot_ctrl/robot_motion_val.h"
#include "robot_params.hpp"
#include <tf/tf.h>
#include "tcp_server.hpp"

#define USE_ROS 1 // debug模式只调试tcp的数据接收，不处理ros问题
// #define DEBUG

//服务器监听套接字
int listenfd;
int confd;
int max_fd;
//创建子线程
pthread_t tid;
//服务端select()函数准备  
fd_set read_fds;
struct timeval timeout;
//检测是否正确连接
int isConnected = 0;
int ps_success = 0;

std::string fan_data_str;

//运动指令消息发布器
ros::Publisher motion_cmd_pub;
ros::Subscriber motion_val_sub;

ros::Timer tcp_recv_timer;

// 位资数据
float pose_data[3] = {0};
geometry_msgs::PoseStamped lvban_pose;



int main(int argc, char** argv)
{
    ros::init(argc, argv, "tcp_server_node");
    ros::NodeHandle nh;
    //初始化ros节点
    #if USE_ROS
    // ros::NodeHandle nh_private("~");
    //消息发布器和订阅器建立
    motion_cmd_pub = nh.advertise<TCP_ROBOT_CMD_TYPE>(TCP_ROBOT_CMD , 1);
    motion_val_sub = nh.subscribe(ROBOT_TCP_VAL, 1, motion_val_callback);
    // fan_data_sub = nh.subscribe("fan_pwm_info", 1, FanDataCallback); 
    // nh_private.param<int>("SERVER_PORT",SERVER_PORT,9527);
    // nh_private.getParam("SERVER_PORT",SERVER_PORT);
    #endif

    std::cout   << YELLOW_STRING << BLOD_STRING 
                << "SERVER_PORT: " << SERVER_PORT 
                << RESET_STRING << std::endl;
    //服务器sockaddr
    struct sockaddr_in servaddr;
    servaddr.sin_family = AF_INET;//IPV4
    servaddr.sin_port = htons(SERVER_PORT); //确定端口号
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //客户端IP
    char client_IP[1024];
    //客户端sockaddr
    struct sockaddr_in client_addr;
    //客户端sockaddr长度
    socklen_t client_addr_len;

    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    

    //整体流程： socket()->bind()->listen()->accept->recv()或者sent()->close()

    //与客户端建立通信 socket->bind->listen->accept
    listenfd = socket(AF_INET, SOCK_STREAM, 0); //AF_INET代表IPV4,SOCK_STREAM代表TCP协议
    //设置socket的SO_REUSEADDR选项，避免端口占用
    int opt = 1;
    setsockopt(listenfd,SOL_SOCKET,SO_REUSEADDR,&opt,sizeof( opt ));

    if (listenfd == -1)//出现错误
    {
        ROS_ERROR("creat socket erro: %s(errno: %d)", strerror(errno), errno);
        return 0;
    }

    // while(ros::ok()) 
    // {
    if ((bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr))) == -1)//bind将socket连接到网卡
    {
        // ROS_ERROR("bind socket erro: %s(errno: %d)", strerror(errno), errno);
        // 将上一行改为cout
        std::cout   << RED_STRING << UNDERLINE_STRING
                    << "bind socket erro: " << strerror(errno) << "(errno: " << errno << ")" 
                    << RESET_STRING << std::endl;
        //continue;
        return 0;
    }
    // }
    if ((listen(listenfd, 128)) == -1)//开始进入监听状态,等待客户端的连接
    {
        // ROS_ERROR("listen socket erro: %s(errno: %d)", strerror(errno), errno);
        // 将上一行改为cout
        std::cout   << RED_STRING << UNDERLINE_STRING
                    << "listen socket erro: " << strerror(errno) << "(errno: " << errno << ")" 
                    << RESET_STRING << std::endl;
        return 0;
    }

    // 初始化文件描述符集合
    FD_ZERO(&read_fds);
    FD_SET(listenfd, &read_fds);
    // 设置最大文件描述符和超时时间
    max_fd = listenfd;
    timeout.tv_sec = 15; // 5秒超时 如果 timeout 为 NULL，则 select 将阻塞，直到有一个文件描述符就绪（即可读、可写或出现异常条件）
    timeout.tv_usec = 0;

    ROS_INFO("======waiting for client's request======");
    ros::Rate r(10);
    while(ros::ok()) 
    {
        if(isConnected==0) //未成功连接
        {
            // 复制文件描述符集合，因为 select 会修改它
            fd_set read_fds_copy = read_fds;
            // 调用 select 函数
            // int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, &timeout);
            int ret = select(max_fd + 1, &read_fds_copy, NULL, NULL, NULL);
            if (ret < 0) { //表示调用失败。此时，errno 被设置为适当的错误码，以指示失败的原因。
                perror("select");
                close(listenfd);
                exit(EXIT_FAILURE);
            } 
            else if (ret == 0) { //如果 timeout 为 NULL，则这种情况不会发生（除非发生错误）。
                printf("Timeout occurred! No data...\n");
                continue; // 超时，没有数据可读，继续循环  
            }
            // 检查监听 socket 是否有新的连接
            if (FD_ISSET(listenfd, &read_fds_copy)) 
            {
                client_addr_len = sizeof(client_addr);
                if((confd = accept(listenfd, (struct sockaddr*)&client_addr, &client_addr_len) )== -1) //accept默认会阻塞进程
                {   //accept接受任意客户端的连接，并返回一个新的socket：confd，以及客户端的IP地址 client_addr 
                    //新的socket：confd负责与客户端通信，而旧的socket，listenfd依旧用于服务端的监听
                    ROS_ERROR("accept socket erro: %s(errno: %d)", strerror(errno), errno); 
                    continue;//不断地accept，直到成功为止
                }
                // 将新的连接添加到文件描述符集合中
                FD_SET(confd, &read_fds);
                if (confd > max_fd) {
                    max_fd = confd; // 更新最大文件描述符
                }
                //显示客户端IP、端口号信息
                ROS_INFO("client_ip:%s port:%d", 
                        inet_ntop(AF_INET, &client_addr.sin_addr.s_addr, client_IP, sizeof(client_IP)),
                        ntohs(client_addr.sin_port));
                isConnected = 1;
            }
        }
        else if(isConnected==1) //已经成功连接
        {
            if(ps_success==0)
            {
                int rett;
                rett = pthread_create(&tid, nullptr, InstructionPubCallback, nullptr);//创建子线程，通过套接字confd接收消息
                if(rett != 0) 
                {
                std::cerr << "thread creat error!" << std::endl;
                }
                else
                {
                    ps_success=1;//成功创建线程
                }
            }
        }
        if(!ros::ok())
        {
            close(confd);//关闭套接字
            close(listenfd);    // 关闭监听 socket
            pthread_cancel(tid);//关闭子线程，不写应该也可以，主线程结束后会退出进程，所以进程里的其他线程都会终止结束
            break;
        }
        // r.sleep();
        ros::spinOnce();
    }

    close(confd);//关闭套接字
    close(listenfd);
    char* s = NULL;
    pthread_join(tid,(void**)&s); //阻塞，保证在退出的时候退出线程
    pthread_cancel(tid);//关闭子线程，不写应该也可以，主线程结束后会退出进程，所以进程里的其他线程都会终止结束
    return 0;//若退出后端口依旧被占用，可以通过 netstat -anp | grep 9527 来查看进程号，然后kill掉
}


//控制指令发布回调函数
void* InstructionPubCallback(void* arg)
{
    while (ros::ok())
    {
        char buf[1024] = "";         //服务器读取字符串，最大1024个字节
        if(recv(confd, buf, sizeof(buf), 0) <= 0) 
         { // 客户端关闭连接或者出错
            if(isConnected==1)
            {
                ROS_INFO("Socket disconnected!");
            }
            isConnected = 0;
            //break;
            continue;
        };
        // 处理接收到的数据，整合成一个字符串容器
        std::vector<std::string> motion_instruction_str;
        // motion_instruction_str容器中已经为分割后的控制字符串
        buf_split(motion_instruction_str, buf, "\t");  
        
        #ifndef DEBUG
        robot_ctrl::tcp_motion_cmd motion_msg;//待发送的tcp指令消息
        

        
        std::string mode;
        mode = motion_instruction_str.at(0);
        std::cout   << BLOD_STRING
                    << GREEN_STRING
                    << "mode: " << mode
                    << RESET_STRING << std::endl;
        
        if( mode ==  ROBOT_STOP || 
            mode ==  ROBOT_CALI ||
            mode ==  ROBOT_TIGHT_EN ||
            mode ==  ROBOT_TIGHT_DIS)
        {
            motion_msg.cmdType = mode;    
        }
        else if(mode == ROBOT_MOTION){
            motion_msg.cmdType = ROBOT_MOTION;
            // 周向速度对应原来“x方向”的位置
            motion_msg.v_cir = atof(motion_instruction_str[1].c_str())/10.0;
            motion_msg.v_axi = atof(motion_instruction_str[2].c_str())/10.0;
            std::cout  << "v_axi: " << motion_msg.v_axi << std::endl;
            std::cout  << "v_cir: " << motion_msg.v_cir << std::endl;
        }
        else if(mode == ROBOT_ANGLE){
            // 软件默认范围为3到9，做一个线性映射
            float angle = atof(motion_instruction_str[1].c_str());
            angle = mechAngleRange[0] + (angle - 3) * (mechAngleRange[1] - mechAngleRange[0]) / (9 - 3);
            motion_msg.cmdType = ROBOT_ANGLE;
            motion_msg.angle_front = angle;
            motion_msg.angle_back = angle;
            std::cout  << "angle_front: " << angle << std::endl;
            std::cout  << "angle_back: " << angle << std::endl;
        }
        motion_cmd_pub.publish(motion_msg);//发布消息
        #else
        // debug模式下只打印处理后的motion_instruction_str数据
        std::cout << "motion_instruction_str: \n";
        for (int i = 0; i < motion_instruction_str.size(); i++)
        {
            std::cout << motion_instruction_str[i] << "\n";
        }
        std::cout << std::endl;
        #endif

    }
    ps_success=0;
    close(confd);
    close(listenfd);
    pthread_exit(NULL);
}

void* MessageSubCallback(void* arg) {
    ros::spin();
    return 0;
}








