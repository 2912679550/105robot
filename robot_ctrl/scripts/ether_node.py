from bottom_control import BottomControl
from packInfo import packInfo
import rospy
import time
import os

class ETHER_NODE():
    def __init__(self, frequncy = 100):
        self.frequncy = frequncy
        # 存储控制板的控制IP与收到的星系
        self.main_assist_ctrl = []
        self.main_assist_info = []
        self.push_ctrl = []
        self.push_info = []

        # todo 创建控制板消息链路，初始化消息接收
        try:
            for i in range(6):
                self.main_assist_ctrl.append(BottomControl(ip = '192.168.110.20%d'%(i) , type = '1', freq = self.frequncy))
                self.main_assist_info.append(packInfo())
            self.push_ctrl = BottomControl(ip = '192.168.110.220', type = '2', freq = self.frequncy)
            self.push_info = packInfo()
        except Exception as err:
            print('机器人初始化失败', err)
            for i in range(6):
                self.main_assist_ctrl[i].tcpCliSock.close()
                time.sleep(1.0)
                os._exit(0)   

if __name__ == "__main__":
    rospy.init_node("ethernet_node", anonymous=True)
    informHandle = ETHER_NODE(100)
    while not rospy.is_shutdown():
        rospy.spin()
    if(rospy.is_shutdown()):
        #task_object.robot_stop()    
        for i in range(3):
            informHandle.steer_ctrl[i].tcpCliSock.close() 
        time.sleep(1.0)
        os._exit(0)   

