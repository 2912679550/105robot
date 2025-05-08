import rospy
import time
import os
import socket
from packInfo import packInfo
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
            full_ip = "192.168.110." + tail_ip
            self.ether_nodes_buf.append(BottomControl(ip = full_ip , port = 5000 ,type = '3', frequency = self.frequency))
            self.ether_info_buf.append(packInfo())
            print('创建控制板 %s 成功' % full_ip)
            time.sleep(1)
            # 开定时器任务，循环接收数据
            self.ether_recv_task = rospy.Timer(rospy.Duration(1.0/self.frequency), self.ether_revc_callback)
            # 创建ROS发布，用来广播32端接收到的信息状态
            self.single_val_pub = rospy.Publisher(self.topic_out , push_board_val , queue_size = 1)
            self.single_cmd_sub = rospy.Subscriber(self.topic_in , push_board_cmd , self.cmd_callback , queue_size = 1)
        except Exception as e:
            print("Error creating UDP socket:", e)


