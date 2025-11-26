#!/bin/bash
###
 # @Author: lxs
 # @Date: 2022-11-11 03:52:08
 # @LastEditTime: 2023-06-03 01:48:45
 # @LastEditors: lxs
 # @Description: 
 # @FilePath: /new_steer/climbRun.sh
 # Copyright (c) 2021 LXScience&Technology. All rights reserved.
### 
sleep 10
# sudo chmod 777 /dev/ttyWCHUSB3
# sudo chmod 777 /dev/ttyUSB0 
DIR=$(cd $(dirname $0);pwd) #获取当前目录
source /opt/ros/melodic/setup.bash
source /home/robot/bot105_ws/devel/setup.bash
# roslaunch steer_track test.launch
roslaunch robot_ctrl robot_all_with_cam.launch
