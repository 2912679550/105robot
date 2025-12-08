# 用于发布控制机器人的虚拟指令

# cd ./..
pwd
source ./devel/setup.zsh

rostopic pub -1 /TCP_ROBOT_CMD_topic robot_ctrl/tcp_cmd_val "cmdType: 'FORWARD'
v_axi: 0.2
v_cir: 0.1
dia_front: 120.0
dia_back: 118.0
dir_tight_front: 0.0
dir_tight_back: 0.0
push_length_f: 10.0
push_length_b: 10.0"