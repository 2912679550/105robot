#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import os
import socket
from packInfo import packInfo
import ros_channel
from bottom_control import BottomControl
from robot_ctrl.msg import push_board_cmd
from robot_ctrl.msg import push_board_val

class PUSH_CMD_VAL:
    # 使用类来定义ROS消息的结构体
    def __init__(self):
        # 分别对应前、后、中三个推杆控制数据，当存储CMD时代表期望值，存储VAL时代表当前值
        self.f_length = 0.0
        self.b_length = 0.0
        self.m_length = 0.0

class PUSH_CTRL:
    def __init__(self , topic_in , topic_out , ip_start , frequency = 100):
        self.frequency = frequency
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.ip_start = ip_start
        
        self.ether_info_buf = []
        self.ether_nodes_buf = []
        self.current_val = PUSH_CMD_VAL()
        self.current_cmd = PUSH_CMD_VAL()
        self.startRecvFlag = False
        
        try:
            tail_ip = str(self.ip_start)
            full_ip = "192.168.0." + tail_ip
            self.ether_nodes_buf.append(BottomControl(ip = full_ip , port = 5000 ,type = '3', frequency = self.frequency))
            self.ether_info_buf.append(packInfo())
            print('创建控制板 %s 成功' % full_ip)
            time.sleep(1)
            # 开定时器任务，循环接收数据
            self.ether_recv_task = rospy.Timer(rospy.Duration(1.0/self.frequency), self.ether_revc_callback)
            # 创建ROS发布，用来广播32端接收到的信息状态
            self.single_val_pub = rospy.Publisher(self.topic_out , push_board_val , queue_size = 1)
            self.single_cmd_sub = rospy.Subscriber(self.topic_in , push_board_cmd , self.cmd_callback , queue_size = 1)
        except Exception as err:
            print('机器人初始化失败', err)
            self.ether_nodes_buf[0].tcpCliSock.close()
            self.fan_ctrl[0].tcpCliSock.close()
            time.sleep(1.0)
            os._exit(0)  
    
    def start_recv(self):
        self.startRecvFlag = True
    
    def stop_recv(self):
        self.startRecvFlag = False
        
    def ether_revc_callback(self , event):
        generat_msg = push_board_val()
        # 先初始化消息防止运行报错
        generat_msg.cur_length_f = 0.0
        generat_msg.cur_length_b = 0.0
        generat_msg.cur_length_m = 0.0
        if(self.startRecvFlag == True):
            self.ether_nodes_buf[0].recvTask(self.ether_info_buf[0])
            generat_msg.cur_length_f = self.ether_info_buf[0].PushValName["f_length"]
            generat_msg.cur_length_b = self.ether_info_buf[0].PushValName["b_length"]
            generat_msg.cur_length_m = self.ether_info_buf[0].PushValName["m_length"]
            # 发布当前值
            self.single_val_pub.publish(generat_msg)

    def cmd_callback(self , msg):
        self.current_cmd.f_length = float(msg.tar_length_f)
        self.current_cmd.b_length = float(msg.tar_length_b)
        self.current_cmd.m_length = float(msg.tar_length_m)
        # 装填到等待发送的info中
        self.ether_info_buf[0].PushCmdName["f_length"] = self.current_cmd.f_length
        self.ether_info_buf[0].PushCmdName["b_length"] = self.current_cmd.b_length
        self.ether_info_buf[0].PushCmdName["m_length"] = self.current_cmd.m_length
        # 发送数据
        self.ether_nodes_buf[0].sendTask(self.ether_info_buf[0])
        
# 主程序
if __name__ == "__main__":
    rospy.init_node('push_node', anonymous=True)
    # 创建一个PUSH_CTRL对象
    push_ctrl = PUSH_CTRL(  ros_channel.PUSH_CMD, 
                            ros_channel.PUSH_VAL, 
                            ros_channel.IP_START_PUSH, 
                            frequency=100)

    while not rospy.is_shutdown():      
        push_ctrl.start_recv()          
        rospy.spin()
    if (rospy.is_shutdown()):
        for i in range(push_ctrl.ether_nodes_buf.__len__()):
            push_ctrl.ether_nodes_buf[i].tcpCliSock.close()
        time.sleep(1.0)
        os._exit(0)

