#!/bin/zsh
###
 # @Author: vulcan
 # @Date: 2022-11-11 03:52:08
 # @LastEditTime: 2023-06-03 01:48:45
 # @LastEditors: vulcan
 # @Description: 
 # @FilePath: /new_steer/climbRun.sh
 # Copyright (c) 2021 LXScience&Technology. All rights reserved.
### 
sleep 5
# sudo chmod 777 /dev/ttyWCHUSB3
# sudo chmod 777 /dev/ttyUSB0 
sudo chmod 777 /dev/cam_ttl
DIR=$(cd $(dirname $0);pwd) #获取当前目录
# source /opt/ros/melodic/setup.bash
source /home/robot/bot105_ws/devel/setup.zsh

ts=$(date "+%Y-%m-%d-%H-%M")
LOG_DIR="${DIR}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/${ts}.txt"

# roslaunch steer_track test.launch
roslaunch robot_ctrl robot_all_with_cam.launch 2>&1 | tee "${LOG_FILE}"
