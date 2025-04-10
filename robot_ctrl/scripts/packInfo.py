class packInfo():
    # boardType = {'1':'Steer', '2':'Adsorption Fan', '3':'Fan', '4':'IO'}
    boardType = {'1':'MainAssist', '2':'Push'}

    # SteerCmdName = {"steer_state" : 1, "dr1_tar_v" : 0.00, "dr2_tar_p" : 0.00, "dr1_tar_c" : 0.00, "ts" : 0.00}
    # SteerValName = {"steer_state" : 0, "dr1_tar_v" : 0.00, "dr1_real_v" : 0.00, "dr1_tar_i" : 0.00, 
    #                 "dr1_real_i" : 0.00, "dr2_tar_p" : 0.00, "dr2_real_p" : 0.00, "dr2_tar_v" : 0.00, 
    #                 "dr2_real_v" : 0.00, "dr2_tar_i" : 0.00, "dr2_real_i" : 0.00, "ts" : 0.00}

        # 新增 MAIN_ASSIST_CMD_NAME 和 MAIN_ASSIST_VAL_NAME 定义
    MainAssistCmdName = {
        "state": 1, 
        "tar_v": 0.00, 
        "tar_p": 0.00, 
        "tar_angle": 0.00, 
        "tar_spring": 0.00
    }

    MainAssistValName = {
        "state": 0, 
        "tar1_v": 0.00, "real1_v": 0.00, 
        "tar2_v": 0.00, "real2_v": 0.00,
        "tar_p": 0.00, "real_p": 0.00, 
        "tar_angle": 0.00, "real_angle": 0.00,
        "tar_spring": 0.00, "real_s1": 0.00, "real_s2": 0.00
    }
    
    PushAssistCmdName = {
    }
    
    PushAssistValName = {
    }    
    # AdsorptionCmdName = {
    #     "active_state" : 1, "s1_tar_p" : 0.00, "s2_tar_p" : 0.00, "s3_tar_p" : 0.00,
    #     "plate_tar_x" : 0.00, "plate_tar_y" : 0.00, "plate_tar_z" : 0.00, "ts" : 0.00}
    # AdsorptionValName = {
    #     "active_state" : 0.00, "s1_tar_p" : 0.00, "s2_tar_p" : 0.00, "s3_tar_p" : 0.00,
    #     "s1_real_p" : 0.00, "s2_real_p" : 0.00, "s3_real_p" : 0.00, "dr1_real_p" : 0.00,
    #     "dr2_real_p" : 0.00, "dr3_real_p" : 0.00, "plate_tar_x" : 0.00, "plate_real_x" : 0.00,
    #     "plate_tar_y" : 0.00, "plate_real_y" : 0.00, "plate_tar_z" : 0.00, "plate_real_z" : 0.00, "ts" : 0.00}

    # FanCmdName = {"fan_state" : 0, "fan_tar_pre" : 0.00, "ts" : 0.00}
    # FanValName = {"fan_state" : 0, "fan_tar_pre" : 0.00, "fan_real_pre" : 0.00, "fan_pwm" : 0.00, "fan_speed" : 0.00, "fan_ts" : 0.00}

    # IOCmdName = {"io_state" : 0, "ts" : 0}
    # IOValName = {"io_state" : 0, "ts" : 0}