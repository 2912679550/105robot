#! /bin/bash
###
 # @Author: lxs
 # @Date: 2023-03-09 02:09:16
 # @LastEditTime: 2023-04-20 04:16:42
 # @LastEditors: lxs
 # @Description: 
 # @FilePath: /new_steer/autoStartConfigure.sh
 # Copyright (c) 2021 LXScience&Technology. All rights reserved.
### 
password="123"
#USB口的kernel,输入udevadm info --attribute-walk --name=/dev/ttyWCHUSB0 | grep KERNELS查看,之后插入串口需保持插入断开不变
usb_kernel="1-2.2:1.0"  
DIR=$(cd $(dirname $0);pwd) #获取当前目录

#赋予sh文件执行权限
echo ${password} | sudo -S chmod u+x ./pipeRun.sh

#设置sh文件开机自启

#编写service文件
SERVICE_FILE="/etc/systemd/system/piperobot.service"
echo ${password} | sudo -S touch $SERVICE_FILE
echo ${password} | sudo -S chmod 777 $SERVICE_FILE
echo ${password} | sudo -S echo "[Unit]" > $SERVICE_FILE
echo ${password} | sudo -S echo "Description = piperobot service" >> $SERVICE_FILE
echo ${password} | sudo -S echo "[Service]" >> $SERVICE_FILE
echo ${password} | sudo -S echo "ExecStart = ${DIR}/pipeRun.sh" >> $SERVICE_FILE
#echo ${password} | sudo -S echo "Restart = always" >> $SERVICE_FILE
echo ${password} | sudo -S echo "Type = simple" >> $SERVICE_FILE
echo ${password} | sudo -S echo "[Install]" >> $SERVICE_FILE
echo ${password} | sudo -S echo "WantedBy = multi-user.target" >> $SERVICE_FILE

echo ${password} | sudo -S systemctl daemon-reload
echo ${password} | sudo -S systemctl enable piperobot.service

# echo ${password} | sudo -S systemctl start piperobot.service

#固定串口号，并设置别名
# USB_RULE_FILE="/etc/udev/rules.d/piperobot.rules"
# echo ${password} | sudo -S touch $USB_RULE_FILE
# echo ${password} | sudo -S chmod 777 $USB_RULE_FILE
# echo ${password} | sudo -S echo KERNEL==\"ttyWCHUSB0\",KERNELS==\"${usb_kernel}\",SUBSYSTEMS==\"usb\",MODE:=\"0777\",SYMLINK+=\"main_serial\" > $USB_RULE_FILE
# echo ${password} | sudo -S echo KERNEL==\"ttyWCHUSB1\",KERNELS==\"${usb_kernel}\",SUBSYSTEMS==\"usb\",MODE:=\"0777\",SYMLINK+=\"debug_serial\" >> $USB_RULE_FILE
# echo ${password} | sudo -S service udev reload
# echo ${password} | sudo -S service udev restart
