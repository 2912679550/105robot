class packInfo():
    def __init__(self):
        # boardType = {'1':'Steer', '2':'Adsorption Fan', '3':'Fan', '4':'IO'}
        self.boardType = {'1': 'MainAssist', '2': 'MainAssist', '3': 'Push'}

        # SteerCmdName = {"steer_state" : 1, "dr1_tar_v" : 0.00, "dr2_tar_p" : 0.00, "dr1_tar_c" : 0.00, "ts" : 0.00}
        # SteerValName = {"steer_state" : 0, "dr1_tar_v" : 0.00, "dr1_real_v" : 0.00, "dr1_tar_i" : 0.00,
        #                 "dr1_real_i" : 0.00, "dr2_tar_p" : 0.00, "dr2_real_p" : 0.00, "dr2_tar_v" : 0.00,
        #                 "dr2_real_v" : 0.00, "dr2_tar_i" : 0.00, "dr2_real_i" : 0.00, "ts" : 0.00}

        # 新增 MAIN_ASSIST_CMD_NAME 和 MAIN_ASSIST_VAL_NAME 定义
        self.MainAssistCmdName = {
            "state": 1,
            "tar_v": 0.00,
            "tar_p": 0.00,
            "tar_angle": 0.00,
            "tar_spring": 0.00
        }

        self.MainAssistValName = {
            "state": 0,
            # "tar1_v": 0.00, 
            "real1_v": 0.00,
            # "tar2_v": 0.00, 
            "real2_v": 0.00,
            # "tar_p": 0.00, 
            "real_p": 0.00,
            # "tar_angle": 0.00, 
            "real_angle": 0.00,
            # "tar_spring": 0.00, 
            "real_s1": 0.00, 
            "real_s2": 0.00,
            "odom_axis": 0.00,
            "odom_cir" : 0.00
        }

        self.PushCmdName = {
            "f_length": 0.00,
            "b_length": 0.00,
            "m_length": 0.00
        }

        self.PushValName = {
            "f_length": 0.00,
            "b_length": 0.00,
            "m_length": 0.00
        }
