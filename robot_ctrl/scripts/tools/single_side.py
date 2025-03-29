import rospy
import time
import os
import socket
from packInfo import packInfo
from bottom_control import BottomControl
# 供给python使用的消息路径(在工作空间中查看）：ls ./devel/lib/python3/dist-packages/robot_ctrl/msg
from robot_ctrl.msg import singel_side_cmd
from robot_ctrl.msg import single_side_val

class SINGLE_SIDE:
    def __init__(self , topic_in , topic_out , ip_start , sub_board_num = 3,frequency = 100):
        self.frequency = frequency
        self.topic_in = topic_in
        self.topic_out = topic_out
        self.ip_start = ip_start
        self.board_num = sub_board_num
        
        self.steer_info = []
        self.steer_ctrl = []
        try:
            for i in range( self.board_num ):
                tail_ip = str(i + ip_start)
                full_ip = '192.168.110.' + tail_ip
                self.steer_ctrl.append( BottomControl(ip = full_ip , type = '1', freq = self.frequency))
                self.steer_info.append(packInfo())
                print('创建控制板 %s 成功' % full_ip)
            self.ether_recv_task = rospy.Timer(rospy.Duration(1.0/self.frequncy), self.ether_revc_callback)
        except Exception as err:
            print('机器人初始化失败', err)
            for i in range(3):
                self.steer_ctrl[i].tcpCliSock.close()
                self.fan_ctrl[i].tcpCliSock.close()
                time.sleep(1.0)
                os._exit(0)   

    def ether_revc_callback(self , event):
        for i in range(self.board_num):
            self.steer_ctrl[i].recvTask(self.steer_info[i])