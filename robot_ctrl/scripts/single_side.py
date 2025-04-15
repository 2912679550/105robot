#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import os
import socket
from packInfo import packInfo
from bottom_control import BottomControl
# 供给python使用的消息路径(在工作空间中查看）：ls ./devel/lib/python3/dist-packages/robot_ctrl/msg
from robot_ctrl.msg import single_side_cmd
from robot_ctrl.msg import single_side_val

class SINGLE_SIDE_VAL:
    # 使用类来定义ROS消息的结构体
    def __init__(self , sub_board_num = 3):
        # 初始化并填充为sub_board_num个0
        self.state = [0] * sub_board_num
        # 轮电机
        self.tar1_v = [0] * sub_board_num
        self.real1_v = [0] * sub_board_num
        # 舵电机
        self.tar2_v = [0] * sub_board_num
        self.real2_v = [0] * sub_board_num
        self.tar_p = [0] * sub_board_num
        self.real_p = [0] * sub_board_num
        # 臂夹角
        self.tar_angle = [0] * 2
        self.real_angle = [0] * 2
        # 弹簧
        self.tar_spring = 0 
        self.real_s1 = 0 
        self.real_s2 = 0

class SINGLE_SIDE_CMD:
    # 使用类来定义接收到的控制指令的结构体
    def __init__(self, sub_board_num = 3):
        # 对应接收到的single_side_cmd.msg
        self.dir_steer_state = [0] * sub_board_num
        self.dir_steer_dir = [0.] * sub_board_num
        self.dir_steer_vel = [0.] * sub_board_num
        self.dir_arm_angle = [0.] * 2
        self.dir_spring_length = 0.0

class SINGLE_SIDE:
    def __init__(self , topic_in , topic_out , ip_start , sub_board_num = 3,frequency = 100):
        self.frequency = frequency
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.ip_start = ip_start
        self.board_num = sub_board_num
        
        self.ether_info_buf = []
        self.ether_nodes_buf = []
        self.current_val = SINGLE_SIDE_VAL()
        self.current_cmd = SINGLE_SIDE_CMD()
        
        try:
            for i in range( self.board_num ):
                tail_ip = str(i + ip_start)
                full_ip = '192.168.0.' + tail_ip
                self.ether_nodes_buf.append( BottomControl(ip = full_ip , port = 5000 ,type = '1', frequency = self.frequency))
                self.ether_info_buf.append(packInfo())
                print('创建控制板 %s 成功' % full_ip)
                # 暂停一小段时间，经过测试不暂停会连接不上
                time.sleep(0.5)
            time.sleep(1)
            # 开定时器任务，循环接收数据
            self.ether_recv_task = rospy.Timer(rospy.Duration(1.0/self.frequency), self.ether_revc_callback)
            # 创建ROS发布，用来广播32端接收到的信息状态
            self.single_val_pub = rospy.Publisher(self.topic_out , single_side_val , queue_size = 1)
            self.single_cmd_sub = rospy.Subscriber(self.topic_in , single_side_cmd , self.cmd_callback , queue_size = 1)
        except Exception as err:
            print('机器人初始化失败', err)
            for i in range(3):
                self.ether_nodes_buf[i].tcpCliSock.close()
                self.fan_ctrl[i].tcpCliSock.close()
                time.sleep(1.0)
                os._exit(0)   

    def ether_revc_callback(self , event):
        generat_msg = single_side_val()
        # 先初始化消息防止运行报错
        generat_msg.cur_steer_state = [0] * 3
        generat_msg.cur_steer_dir = [0.] * 3
        generat_msg.cur_steer_vel = [0.] * 3
        generat_msg.cur_arm_angle = [0.] * 2
        generat_msg.cur_spring_length = [0.] * 2
        # 先接收信息，并存储通用的舵轮信息
        for i in range(self.board_num):
            self.ether_nodes_buf[i].recvTask(self.ether_info_buf[i])
            # * 解析收到的32数据，并分类装填到内部存储中
            generat_msg.cur_steer_state[i] = self.current_val.state[i] = int(self.ether_info_buf[i].MainAssistValName["state"])
            # 舵电机
            self.current_val.tar_p[i] = self.ether_info_buf[i].MainAssistValName["tar_p"]
            generat_msg.cur_steer_dir[i] = self.current_val.real_p[i] = self.ether_info_buf[i].MainAssistValName["real_p"]
            self.current_val.tar2_v[i] = self.ether_info_buf[i].MainAssistValName["tar2_v"]
            self.current_val.real2_v[i] = self.ether_info_buf[i].MainAssistValName["real2_v"]
            # 轮电机
            self.current_val.tar1_v[i] = self.ether_info_buf[i].MainAssistValName["tar1_v"]
            generat_msg.cur_steer_vel[i] = self.current_val.real1_v[i] = self.ether_info_buf[i].MainAssistValName["real1_v"]
        # * 以下的量需要根据实际情况筛选存储，如臂夹角只有辅助控制板才有，而弹簧长度只有主控制板才有
        if self.board_num > 1:  # 多板子时说明为实际使用状态，测试时会将board_num设置为1
            # 继续处理夹角信息，左右辅助控制板的编号分别为1,2
            for i in range(2):
                # 夹角
                self.current_val.tar_angle[i] = self.ether_info_buf[i].MainAssistValName["tar_angle"]
                generat_msg.cur_arm_angle[i] = self.current_val.real_angle[i] = self.ether_info_buf[i].MainAssistValName["real_angle"]
            # 处理弹簧信息（主控板）
            self.current_val.tar_spring = self.ether_info_buf[2].MainAssistValName["tar_spring"]
            generat_msg.cur_spring_length[0] = self.current_val.real_s1 = self.ether_info_buf[2].MainAssistValName["real_s1"]
            generat_msg.cur_spring_length[1] = self.current_val.real_s2 = self.ether_info_buf[2].MainAssistValName["real_s2"]
        # 发布ROS消息
        self.single_val_pub.publish(generat_msg)
        # 打印generat_msg
        # print('========================================== ')
        # print('state: ',self.current_val.state)
        # print('tar1_v: ' ,self.current_val.tar1_v)
        # print('real1_v: ',self.current_val.real1_v)
        # print('tar2_v: ',self.current_val.tar2_v)
        # print('real2_v: ',self.current_val.real2_v)
        # print('tar_p: ',self.current_val.tar_p)
        # print('real_p: ',self.current_val.real_p)
        # print('tar_angle: ',self.current_val.tar_angle)
        # print('real_angle: ',self.current_val.real_angle)
        # print('tar_spring: ',self.current_val.tar_spring)
        # print('real_s1: ',self.current_val.real_s1)
        # print('real_s2: ',self.current_val.real_s2)
        # print('========================================== ')

    def cmd_callback(self , msg):
        # 解析ROS消息，这里接收到的是舵轮的朝向与速度信息、期望的运行状态等
        for i in range(self.board_num):
            self.current_cmd.dir_steer_state[i] = int(msg.dir_steer_state[i])
            self.current_cmd.dir_steer_dir[i] = float(msg.dir_steer_dir[i])
            self.current_cmd.dir_steer_vel[i] = float(msg.dir_steer_vel[i])
            # 装填到等待发送的info中
            self.ether_info_buf[i].MainAssistCmdName["state"] = self.current_cmd.dir_steer_state[i]
            self.ether_info_buf[i].MainAssistCmdName["tar_v"] = self.current_cmd.dir_steer_vel[i]
            self.ether_info_buf[i].MainAssistCmdName["tar_p"] = self.current_cmd.dir_steer_dir[i]
        if self.board_num > 1:
            # 夹角
            self.current_cmd.dir_arm_angle[0] = float(msg.dir_arm_angle[0])
            self.current_cmd.dir_arm_angle[1] = float(msg.dir_arm_angle[1])
            self.ether_info_buf[0].MainAssistCmdName["tar_angle"] = self.current_cmd.dir_arm_angle[0]
            self.ether_info_buf[1].MainAssistCmdName["tar_angle"] = self.current_cmd.dir_arm_angle[1]
            
            # 弹簧
            self.current_cmd.dir_spring_length = float(msg.dir_spring_length)
            self.ether_info_buf[2].MainAssistCmdName["tar_spring"] = self.current_cmd.dir_spring_length
        # 发送信息
        for i in range(self.board_num):
            self.ether_nodes_buf[i].sendTask(self.ether_info_buf[i])
            # print('发送控制指令到 %s' % self.ether_nodes_buf[i].ip)
            # print('dir_steer_state: ',self.current_cmd.dir_steer_state[i])
            # print('dir_steer_dir: ',self.current_cmd.dir_steer_dir[i])
            # print('dir_steer_vel: ',self.current_cmd.dir_steer_vel[i])

# 测试用主程序
# if __name__ == "__main__":
#     rospy.init_node("single_side_node", anonymous=False)
#     # 创建一个SINGLE_SIDE对象
#     single_side = SINGLE_SIDE(topic_in = '/robot/single_side_cmd', topic_out = '/robot/single_side_val', ip_start = 201, sub_board_num = 3 , frequency = 100)
#     while not rospy.is_shutdown():                
#         rospy.spin() 
#     if(rospy.is_shutdown()):
#         for i in range(single_side.ether_nodes_buf.__len__()):
#             single_side.ether_nodes_buf[i].tcpCliSock.close() 
#         time.sleep(1.0)
#         os._exit(0)