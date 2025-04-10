#!/usr/bin/env python3
# coding:utf-8
from single_side import SINGLE_SIDE
import rospy
import ros_channel
import time
import os

if __name__ == "__main__":
    rospy.init_node("front_side_node", anonymous=False)
    # 创建一个SINGLE_SIDE对象
    single_side = SINGLE_SIDE(  topic_in = ros_channel.ROBOT_STM_CMD_F, 
                                topic_out = ros_channel.STM_ROBOT_VAL_F, 
                                ip_start = 201, 
                                sub_board_num = 3 , 
                                frequency = 100)
    while not rospy.is_shutdown():                
        rospy.spin() 
    if(rospy.is_shutdown()):
        for i in range(single_side.ether_nodes_buf.__len__()):
            single_side.ether_nodes_buf[i].tcpCliSock.close() 
        time.sleep(1.0)
        os._exit(0)
