# ether node 接收到的，注意这时就都是真实的值了
int32[3]   cur_steer_state      # 舵轮运行状态
float32[3] cur_steer_dir        # 舵轮朝向
float32[3] cur_steer_vel        # 舵轮速度
float32[2] cur_arm_angle        # 当前的单侧一臂与二臂夹角
float32[2] cur_spring_length    # 当前的左右弹簧长度
float32[3] odom_axis            # 左、右、中三轮形成的当前里程计信息
float32[3] odom_cir       





