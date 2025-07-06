#!/usr/bin/env python3    
import rospy
from serialize_data import serializeData
from serial_write import send_to_serial
from camera_ctrl.msg import Cameractrl



def camera_callback(msg):
    try:
        serialized_str = None
        if msg.cmdType == 'CameraPose':
            serialized_str = bytearray.fromhex(serializeData(msg.yaw, msg.roll, msg.pitch))
        elif msg.cmdType == 'CameraCmd':
            if msg.command == 'yawlock':
                serialized_str = bytearray.fromhex("aa 06 05 01 01 18")
            elif msg.command == 'alllock':
                serialized_str = bytearray.fromhex("aa 06 05 01 00 cd")
            elif msg.command == 'black':
                serialized_str = bytearray.fromhex("aa 06 01 07 02 fb")
            elif msg.command == 'rgb':
                serialized_str = bytearray.fromhex("aa 06 01 07 01 51")
            elif msg.command == 'vediorecord':
                serialized_str = bytearray.fromhex("aa 05 01 05 df")
        
        if serialized_str:
            send_to_serial(serialized_str)
            print(f"Serialized data: {serialized_str}")
        else:
            rospy.logwarn("Unsupported command received")
            
    except Exception as e:
        rospy.logerr(f"Error processing message: {e}")
rospy.init_node('Camera_subscriber')
rospy.Subscriber('/CAMERA_CMD_topic', Cameractrl, camera_callback)
rospy.spin()
