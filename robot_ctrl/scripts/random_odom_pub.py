#!/usr/bin/env python3
import rospy
import random
from robot_ctrl.msg import robot_motion_val

if __name__ == "__main__":
    rospy.init_node("random_odom_pub")
    pub = rospy.Publisher("/robot_tcp_val_topic", robot_motion_val, queue_size=10)
    rate_hz = rospy.get_param("~rate", 100)  # 可通过rosparam设置频率，默认10Hz
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        msg = robot_motion_val()
        msg.odom_pos = [random.uniform(-10, 10), random.uniform(-10, 10)]
        pub.publish(msg)
        # rospy.loginfo("Published odom_pos: %s", msg.odom_pos)
        rate.sleep()
