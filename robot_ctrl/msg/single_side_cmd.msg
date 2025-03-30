# 用于直接发给机器人单侧控制板的控制指令，对应32端的MAIN_ASSIST控制板，这里是指期望值
int32[3]    dir_steer_state     # 舵轮运行状态
float32[3]  dir_steer_dir       # 舵轮朝向
float32[3]  dir_steer_vel       # 舵轮速度
float32[2]  dir_arm_angle       # 期望的单侧一臂与二臂夹角
float32     dir_spring_length   # 期望的弹簧长度





