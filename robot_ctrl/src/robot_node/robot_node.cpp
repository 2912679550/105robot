#include "robot_node.hpp"
#include "robot_params.hpp"

// ! ========================== main robot ===========================
MAIN_ROBOT::MAIN_ROBOT(ros::NodeHandle* nh_ ){
    if(nh_ == nullptr){
        nh_ = new ros::NodeHandle();
        std::cout<< YELLOW_STRING << BLOD_STRING << "nh_ is nullptr" << RESET_STRING << std::endl;
    }
    //* 初始化内置控制器
    front_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_F, STM_ROBOT_VAL_F, nh_);
    back_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_B, STM_ROBOT_VAL_B, nh_);
    push_ctrl_ = new PUSH_CTRL(PUSH_CMD , PUSH_VAL, nh_);
    pipe_controller_ = new PipeController();
    pose_closed_ctrl_ = new POSE_CLOSED_LOOP(IMU_FRONT, IMU_BACK, front_side_, back_side_, nh_);
    motion_planner_ = new MOTION_PLAN();

    front_side_->odom_handler_ = new MICRO_ODOM(odomValueCoeff_f);  // 前侧里程计处理类
    back_side_->odom_handler_ = new MICRO_ODOM(odomValueCoeff_b);

    // 创建手柄消息的订阅与回传发布者
    tcp_pub_ = nh_->advertise<ROBOT_TCP_VAL_TYPE>(ROBOT_TCP_VAL, 1);
    tcp_sub_ = nh_->subscribe(TCP_ROBOT_CMD , 1, &MAIN_ROBOT::motion_cmd_callback, this);
    std::cout<< GREEN_STRING << BLOD_STRING <<UNDERLINE_STRING<< "main robot node start" << RESET_STRING << std::endl;
}

MAIN_ROBOT::~MAIN_ROBOT(){
    if(front_side_ != nullptr){
        delete front_side_;
        front_side_ = nullptr;
    }
    if(back_side_ != nullptr){
        delete back_side_; 
        back_side_ = nullptr;
    }
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
    if(push_ctrl_ != nullptr){
        delete push_ctrl_;
        push_ctrl_ = nullptr;
    }
}

void MAIN_ROBOT::cmd_hand_maked(TCP_ROBOT_CMD_TYPE* msg){
    // 直接调用回调函数，模拟接收数据
    if(msg != nullptr){
        TCP_ROBOT_CMD_CPTR msg_ptr(new TCP_ROBOT_CMD_TYPE(*msg));
        motion_cmd_callback(msg_ptr);
    }
    else{
        std::cout<< RED_STRING << "robot node receive hand made command is nullptr" << RESET_STRING << std::endl;
    }
}

void MAIN_ROBOT::change_state(ROBOT_STATE new_state){
    if(new_state != robot_state_){
        pre_robot_state_ = robot_state_;
        robot_state_ = new_state;
        cur_state_repeat_count_ = 0;
    }else{
        cur_state_repeat_count_++;  // 输入的状态与当前状态相同，重复按下次数加1
    }
}

void MAIN_ROBOT::motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg){
    if(msg != nullptr)
    {
        // 处理接收到的数据，整合成一个字符串容器
        std::string mode = msg->cmdType;
        std::cout<< BLUE_STRING << BLINK_STRING << "receive command: " << mode << RESET_STRING << std::endl;
        // todo 基本运动与功能控制
        if(mode == ROBOT_STOP){
            front_side_->set_steer(steerState::STOP); back_side_->set_steer(steerState::STOP);
            motion_planner_->reset_motion();
            front_side_->pipe_sped_diff(false);
            back_side_->pipe_sped_diff(false);
            change_state(ROBOT_STATE::MOTION_STOP);
        }
        else if(mode == ROBOT_CALI){
            front_side_->set_steer(steerState::RESET); back_side_->set_steer(steerState::RESET);
            change_state(ROBOT_STATE::MOTION_STOP);
        }
        else if(mode == ROBOT_MOTION){
            // 只有stop或手操模式才可以进入手操遥控模式
            if(robot_state_ == ROBOT_STATE::MOTION_STOP || robot_state_ == ROBOT_STATE::NORMAL_HAND_CTRL){
                tar_v_aix_ = msg->v_axi;
                tar_v_cir_ = msg->v_cir;
                // 重置使能标志位
                pipe_cali_flag_ = false;    // 自动进弯与出弯重置
                motion_planner_->reset_motion();
                change_state(ROBOT_STATE::NORMAL_HAND_CTRL);
            }else{
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Cannot enter normal hand control , please stop first"
                    << RESET_STRING << std::endl;
            }
        }
        else if(mode == ROBOT_STEP){
            // 步进运动
            if(robot_state_ != ROBOT_STATE::MOTION_STOP){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Cannot enter step motion , please stop first"
                    << RESET_STRING << std::endl;
                return;
            }
            // 设置运动范围
            motion_planner_->set_motion_range(robot_axis_odom_ , robot_cir_odom_,msg->v_axi, msg->v_cir, MOTION_PLAN::MOTION_MODE::STEP);
            change_state(ROBOT_STATE::PLAN_MOTION);
        }
        else if(mode == ROBOT_SCAN){
            if(robot_state_ != ROBOT_STATE::MOTION_STOP){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Cannot enter scan motion , please stop first"
                    << RESET_STRING << std::endl;
                return;
            }
            // 设置运动范围
            motion_planner_->set_motion_range(robot_axis_odom_ , robot_cir_odom_,msg->v_axi, msg->v_cir, MOTION_PLAN::MOTION_MODE::SCAN);
            change_state(ROBOT_STATE::PLAN_MOTION);
        }
        // todo IMU姿态闭环相关
        else if(mode == ROBOT_ON_POSE) pose_closed_ctrl_->turn_on_close_loop_(); // 关闭姿态闭环
        else if(mode == ROBOT_OFF_POSE) pose_closed_ctrl_->turn_off_close_loop_(); // 开启姿态闭环
        // todo 夹紧控制
        else if(mode == ROBOT_T_L_F) front_side_->set_tight(msg->dir_tight_front);  // 前侧夹紧长度控制
        else if(mode == ROBOT_T_L_B) back_side_->set_tight(msg->dir_tight_back);  // 后侧夹紧长度控制
        // todo 变形控制
        else if(mode == ROBOT_DIA){ 
            front_side_->set_dia(msg->dia_front); back_side_->set_dia(msg->dia_back); 
            saved_pipe_r_[0] = msg->dia_front / 2.0f;  saved_pipe_r_[1] = msg->dia_back / 2.0f;
        }
        else if(mode == ROBOT_BOTH_LENGTH){ push_ctrl_->set_body_length(msg->push_length_f, msg->push_length_b); }
        // todo 自动进弯
        else if(mode == AUTO_PIPE_CALI){
            if(pipe_cali_flag_ == false){
                save_odom_and_imu(); // 保存当前里程计与IMU数据
                pipe_cali_flag_ = true; // 开启自动进弯标定使
            }
            front_side_->pipe_sped_diff(true , saved_pipe_r_[0]);
            back_side_->pipe_sped_diff(true , saved_pipe_r_[1]);
        } 
        else if(mode == AUTO_IN_PIPE){
            if(pipe_cali_flag_ == false){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Please calibrate first before entering pipe"
                    << RESET_STRING << std::endl;
                return;
            }
            // 自动进弯
            if(robot_state_!= ROBOT_STATE::MOTION_STOP){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Cannot enter auto in pipe , please stop first"
                    << RESET_STRING << std::endl;
                return;
            }
            change_state(ROBOT_STATE::IN_PIPE);
            // 进弯时以前侧里程计为准
            pipe_controller_->set_start_(saved_odom_aix_[0], saved_odom_cir_[0], &(saved_quat_[0]) );
            pipe_controller_->set_geometry_params_(300.0 , saved_pipe_r_[0]);
        }
        else if(mode == AUTO_OUT_PIPE){
            if(pipe_cali_flag_ == false){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Please calibrate first before exiting pipe"
                    << RESET_STRING << std::endl;
                return;
            }
            // 自动出弯
            if(robot_state_ != ROBOT_STATE::MOTION_STOP){
                std::cout<< YELLOW_STRING << BLOD_STRING 
                    << "Cannot enter auto out pipe , please stop first"
                    << RESET_STRING << std::endl;
                return;
            }
            change_state(ROBOT_STATE::OUT_PIPE);
            // 出弯时以后侧里程计为准
            pipe_controller_->set_start_(saved_odom_aix_[1], saved_odom_cir_[1], &(saved_quat_[1]) );
            pipe_controller_->set_geometry_params_(300.0 , saved_pipe_r_[1]);
        }

        else{
            std::cout<< RED_STRING << "robot node receive unknown command" << RESET_STRING << std::endl;
        }
    }
}

void MAIN_ROBOT::pubCmd(){
    front_side_->pub_cmd();
    back_side_->pub_cmd();
    push_ctrl_->pub_cmd();
};

void MAIN_ROBOT::robot_ctrl(bool printFlag){
    // * 执行一些状态机指令
    front_side_->single_side_ctrl();  // 前侧单侧控制逻辑
    back_side_->single_side_ctrl();   // 后侧单侧控制逻辑
    odom_handler(printFlag); // 处理里程计数据

    // * 提前配置一些switch中可能产生的变量
    float v_aix = 0.0f;
    float v_cir = 0.0f;
    float push_length = 0.0f;
    steerState plan_state = steerState::STOP;

    switch(robot_state_){
        case ROBOT_STATE::MOTION_STOP:
            // * 停止状态
            motion_planner_->reset_motion();
            tar_v_aix_ = 0.0f;
            tar_v_cir_ = 0.0f;
            break;
        case ROBOT_STATE::NORMAL_HAND_CTRL:
            // * 正常运动模式
            pose_closed_ctrl_->close_loop_pid_(printFlag); // 姿态闭环PID计算
            front_side_->set_steer(steerState::NORMAL , tar_v_aix_ , tar_v_cir_,
                                    pose_closed_ctrl_->pid_out_p_ , pose_closed_ctrl_->pid_out_y_);
            back_side_->set_steer(steerState::NORMAL , tar_v_aix_ , tar_v_cir_,
                                    pose_closed_ctrl_->pid_out_p_ , pose_closed_ctrl_->pid_out_y_);
            
            break;
        case ROBOT_STATE::PLAN_MOTION:
            plan_state = motion_planner_->motion_plan_(&v_aix , &v_cir, robot_axis_odom_ , robot_cir_odom_, printFlag);
            front_side_->set_steer(plan_state , v_aix , v_cir);
            back_side_->set_steer(plan_state , v_aix , v_cir);

            if(plan_state == steerState::STOP){
                change_state(ROBOT_STATE::MOTION_STOP);
                std::cout<< GREEN_STRING << BLOD_STRING << "motion plan finished , now stop" << RESET_STRING << std::endl;
            }
            break;
        case ROBOT_STATE::IN_PIPE:
            if (
                pipe_controller_->auto_in_pipe_(robot_axis_odom_, robot_cir_odom_,
                        &(pose_closed_ctrl_->front_imu_handler_->quat_cur), 0.02, printFlag)
            ){
                front_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->assist_wheel_speed_);
                if(pipe_controller_->solve_end_ == true){
                    std::cout<< GREEN_STRING << BLOD_STRING << "pipe auto in solve end" << RESET_STRING << std::endl;
                    back_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->assist_wheel_speed_);
                }else{
                    back_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->main_wheel_speed_);
                    push_ctrl_->set_body_length(pipe_controller_->push_length_, true);
                    push_ctrl_->set_body_length(pipe_controller_->push_length_, false);
                }
            }
            break;
        case ROBOT_STATE::OUT_PIPE:
            if (
                pipe_controller_->auto_out_pipe_(robot_axis_odom_, robot_cir_odom_,
                        &(pose_closed_ctrl_->back_imu_handler_->quat_cur), 0.02, printFlag)
            ){
                back_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->assist_wheel_speed_);
                if(pipe_controller_->solve_end_ == true){
                    std::cout<< GREEN_STRING << BLOD_STRING << "pipe auto out solve end" << RESET_STRING << std::endl;
                    front_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->assist_wheel_speed_);
                }else{
                    front_side_->set_main_assist_speed_(pipe_controller_->main_wheel_speed_, pipe_controller_->main_wheel_speed_);
                    push_ctrl_->set_body_length(pipe_controller_->push_length_, true);
                    push_ctrl_->set_body_length(pipe_controller_->push_length_, false);
                }
            }
            break;
        default:
            break;
    }
    pubCmd();
    // 发布手柄回传数据
    ROBOT_TCP_VAL_TYPE tcp_val;
    tcp_val.push_length[0] = push_ctrl_->cmd_data_.tar_length_f;  // 前侧推杆长度
    tcp_val.push_length[1] = push_ctrl_->cmd_data_.tar_length_b;  // 后侧推杆长度
    tcp_val.front_odom[0] = front_side_->odom_handler_->odom_axis[2];
    tcp_val.front_odom[1] = front_side_->odom_handler_->odom_cir[2];
    tcp_val.back_odom[0] = back_side_->odom_handler_->odom_axis[2];
    tcp_val.back_odom[1] = back_side_->odom_handler_->odom_cir[2];
    tcp_pub_.publish(tcp_val);
}

void MAIN_ROBOT::odom_handler(bool printFlag){
    // todo 里程计逻辑处理
    bool odomPrintFlag = false;  // 是否打印里程计数据
    // * 根据夹紧状态自动控制里程计启停
    // 总体概括为：夹紧状态为false，则暂停里程计；
    // 夹紧状态为true，则判断是否已经夹紧了超过1s，如果超过1s，则开始里程计；否则暂停里程计
    if(front_side_ -> tarTightFlag_ == false){
        front_side_ -> odom_handler_ -> pause_odom();  // 如果前侧夹紧状态为false，则暂停前侧里程计
        front_odom_en_ = false;  // 前侧里程计使能为false
    }else{
        if(front_side_ -> tight_timer_->get_sec() < 1.0f){
            front_side_ -> odom_handler_ -> pause_odom();  // 如果前侧夹紧状态为false，则暂停前侧里程计
            front_odom_en_ = false;  // 前侧里程计使能为false
        }else{
            front_side_ -> odom_handler_ -> start_odom();  // 如果前侧夹紧状态为true，则开始前侧里程计
            front_odom_en_ = true;  // 前侧里程计使能为true
        }
    }
    // 后侧里程计逻辑处理
    if(back_side_ -> tarTightFlag_ == false){
        back_side_ -> odom_handler_ -> pause_odom();  // 如果后侧夹紧状态为false，则暂停后侧里程计
        back_odom_en_ = false;  // 后侧里程计使能为false
    }else{
        if(back_side_ -> tight_timer_->get_sec() < 1.0f){
            back_side_ -> odom_handler_ -> pause_odom();  // 如果后侧夹紧状态为false，则暂停后侧里程计
            back_odom_en_ = false;  // 后侧里程计使能为false
        }else{
            back_side_ -> odom_handler_ -> start_odom();  // 如果后侧夹紧状态为true，则开始后侧里程计
            back_odom_en_ = true;  // 后侧里程计使能为true  
        }
    }

    // * 根据里程计启停状态，生成里程计数据
    if(front_odom_en_ && back_odom_en_){
        // 如果前后两侧里程计都使能，则取两侧平均值
        robot_axis_odom_ = (front_side_->odom_handler_->odom_axis_avg + back_side_->odom_handler_->odom_axis_avg) / 2.0f;
        robot_cir_odom_ = (front_side_->odom_handler_->odom_cir_avg + back_side_->odom_handler_->odom_cir_avg) / 2.0f;
        odomPrintFlag = true;  // 设置打印标志位为true
    }
    else if(front_odom_en_ && (!back_odom_en_)){
        // 如果只有前侧里程计使能，则取前侧里程计值，并使用前侧里程计刷新后侧里程计
        robot_axis_odom_ = front_side_->odom_handler_->odom_axis_avg;
        robot_cir_odom_ = front_side_->odom_handler_->odom_axis_avg;
        back_side_->odom_handler_->set_cur_val(front_side_->odom_handler_->odom_axis, front_side_->odom_handler_->odom_cir);  // 刷新后侧里程计
        odomPrintFlag = true;  // 设置打印标志位为true
    }
    else if((!front_odom_en_) && back_odom_en_){
        // 如果只有后侧里程计使能，则取后侧里程计值，并使用后侧里程计刷新前侧里程计
        robot_axis_odom_ = back_side_->odom_handler_->odom_axis_avg;
        robot_cir_odom_ = back_side_->odom_handler_->odom_cir_avg;
        front_side_->odom_handler_->set_cur_val(back_side_->odom_handler_->odom_axis, back_side_->odom_handler_->odom_cir);  // 刷新前侧里程计
        odomPrintFlag = true;
    }
    else{
        // 如果两侧里程计都未使能，则将轴向和周向里程计数据置为0
        robot_axis_odom_ = 0.0f;
        robot_cir_odom_ = 0.0f;
    }
    // 打印输出
    if(printFlag && odomPrintFlag){
        std::cout<< YELLOW_STRING << BLOD_STRING << UNDERLINE_STRING 
            << "robot odom (axis , cir): " << RESET_STRING
            << GREEN_STRING
            << robot_axis_odom_ 
            << " , " << robot_cir_odom_ << RESET_STRING << std::endl;
    }
}

void MAIN_ROBOT::save_odom_and_imu(){
    saved_odom_aix_[0] = front_side_->odom_handler_->odom_axis[2];  // 存储前侧主动轮的轴向里程计
    saved_odom_aix_[1] = back_side_->odom_handler_->odom_axis[2];   // 存储后侧主动轮的轴向里程计
    saved_odom_cir_[0] = front_side_->odom_handler_->odom_cir[2];
    saved_odom_cir_[1] = back_side_->odom_handler_->odom_cir[2];
    if(pose_closed_ctrl_->front_imu_handler_ != nullptr) saved_quat_[0] = pose_closed_ctrl_->front_imu_handler_->quat_cur;
    if(pose_closed_ctrl_->back_imu_handler_ != nullptr) saved_quat_[1] = pose_closed_ctrl_->back_imu_handler_->quat_cur;
}

// ! ========================== Pose Closed Loop ===========================

/**
 * @brief 创建姿态闭环的控制器
 * 
 * @param front_imu_topic   前侧IMU话题名称
 * @param back_imu_topic    后侧IMU话题名称
 * @param front_handle      前侧控制器指针
 * @param back_handle       后侧控制器指针   
 * @param nh                ROS节点句柄指针
 */
POSE_CLOSED_LOOP::POSE_CLOSED_LOOP(std::string front_imu_topic , std::string back_imu_topic , 
                                    SINGLE_SIDE_CTRL * front_handle , SINGLE_SIDE_CTRL * back_handle ,
                                    ros::NodeHandle* nh)
{
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
        std::cout<< YELLOW_STRING << BLOD_STRING << "nh_ is nullptr" << RESET_STRING << std::endl;
    }else{
        nh_ = nh;
    }
    front_handle_ = front_handle;
    back_handle_ = back_handle;

    front_imu_handler_ = new IMU_HANDLER(front_imu_topic, nh_);
    front_imu_handler_->imu_robot_matrix = new tf::Matrix3x3(IMU_FRONT_ROTATE[0][0], IMU_FRONT_ROTATE[0][1], IMU_FRONT_ROTATE[0][2],
                                                            IMU_FRONT_ROTATE[1][0], IMU_FRONT_ROTATE[1][1], IMU_FRONT_ROTATE[1][2],
                                                            IMU_FRONT_ROTATE[2][0], IMU_FRONT_ROTATE[2][1], IMU_FRONT_ROTATE[2][2]);
    back_imu_handler_ = new IMU_HANDLER(back_imu_topic, nh_);
    back_imu_handler_->imu_robot_matrix = new tf::Matrix3x3(IMU_BACK_ROTATE[0][0], IMU_BACK_ROTATE[0][1], IMU_BACK_ROTATE[0][2],
                                                        IMU_BACK_ROTATE[1][0], IMU_BACK_ROTATE[1][1], IMU_BACK_ROTATE[1][2],
                                                        IMU_BACK_ROTATE[2][0], IMU_BACK_ROTATE[2][1], IMU_BACK_ROTATE[2][2]);
    // * 配置PID计算器参数
    PID_PARAM pid_params;
    pid_params.p = imu_pitch_P[0];
    pid_params.i = imu_pitch_I[0];
    pid_params.d = imu_pitch_D[0];
    pid_params.Iband = imu_pitch_Iband[0];
    pid_params.outMin = imu_pitch_outRange[0];
    pid_params.outMax = imu_pitch_outRange[1];
    pid_params.outIMin = imu_pitch_outRange[0];
    pid_params.outIMax = imu_pitch_outRange[1];
    pid_params.ts = 1.0f / float(TS);  // 设置采样周期
    pid_pitch_ = new Pid(&pid_params);

    pid_params.p = imu_yaw_P[0];
    pid_params.i = imu_yaw_I[0];
    pid_params.d = imu_yaw_D[0];
    pid_params.Iband = imu_yaw_Iband[0];
    pid_params.outMin = imu_yaw_outRange[0];
    pid_params.outMax = imu_yaw_outRange[1];
    pid_params.outIMin = imu_yaw_outRange[0];
    pid_params.outIMax = imu_yaw_outRange[1];
    pid_params.ts = 1.0f / float(TS);  // 设置采样周期
    pid_yaw_ = new Pid(&pid_params);
}

POSE_CLOSED_LOOP::~POSE_CLOSED_LOOP(){
    if(front_imu_handler_ != nullptr){
        delete front_imu_handler_;
        front_imu_handler_ = nullptr;
    }
    if(back_imu_handler_ != nullptr){
        delete back_imu_handler_;
        back_imu_handler_ = nullptr;
    }
    if(pid_pitch_ != nullptr){
        delete pid_pitch_;
        pid_pitch_ = nullptr;
    }
    if(pid_yaw_ != nullptr){
        delete pid_yaw_;
        pid_yaw_ = nullptr;
    }
}

/**
 * @brief 一键开启姿态闭环控制，执行本函数会同时存储前后imu的各自数据，在后续PID计算时自动选择并使用
 * 
 */
void POSE_CLOSED_LOOP::turn_on_close_loop_(){
    enable_close_loop_ = true;  // 开启姿态闭环控制
    pid_yaw_->Reset();  // 重置偏航角PID控制器
    pid_pitch_->Reset();  // 重置俯仰角PID控制器
    if(front_imu_handler_ != nullptr){
        front_imu_handler_->fix_quat();  // 固定前侧IMU的四元数
    }
    if(back_imu_handler_ != nullptr){
        back_imu_handler_->fix_quat();  // 固定后侧IMU的四元数
    }
}

void POSE_CLOSED_LOOP::turn_off_close_loop_(){
    pid_yaw_->Reset();  // 重置偏航角PID控制器
    pid_pitch_->Reset();  // 重置俯仰角PID控制器
    enable_close_loop_ = false;  // 关闭姿态闭环控制
}

void POSE_CLOSED_LOOP::close_loop_pid_(bool printFlag){
    if(enable_close_loop_ == false ||
        (front_handle_->tarTightFlag_ ==  back_handle_->tarTightFlag_ ) ) 
    {
        // 如果姿态闭环控制未开启，或者前后两侧夹紧状态相同（都为夹紧或都为松开），则不进行姿态闭环控制
        pid_out_p_ = 0.0f;  // 如果姿态闭环控制未开启，则PID输出为0
        pid_out_y_ = 0.0f;
        pid_yaw_->Reset();  // 重置偏航角PID控制器
        pid_pitch_->Reset();  // 重置俯仰角PID控制器
        return;
    }
    IMU_POSE aixs_err;
    // 如果前侧夹紧为true，根据前文逻辑此时后侧一定为false，则使用前侧IMU数据进行姿态闭环控制
    if(front_handle_->tarTightFlag_ == true) front_imu_handler_->get_aixs_err(&aixs_err, printFlag);
    // 如果后侧夹紧为true，根据前文逻辑此时前侧一定为false，则使用后侧IMU数据进行姿态闭环控制
    else back_imu_handler_->get_aixs_err(&aixs_err, printFlag);

    pid_out_p_ = pid_pitch_->Tick(aixs_err.pitch, printFlag);
    pid_out_y_ = pid_yaw_->Tick(aixs_err.yaw, printFlag);

    // * 最后判断全局硬性使能标志位
    if(use_imu_pitch == false) pid_out_p_ = 0.0f;  // 如果不使用IMU俯仰角控制，则PID输出为0
    if(use_imu_yaw == false) pid_out_y_ = 0.0f;  // 如果不使用IMU偏航角控制，则PID输出为0
    return;
}


// ! ========================== Motion Plan ===========================
/**
 * @brief 配置机器人简单运动规划的范围以及模式
 * 
 * @param cur_aix  运动规划起点轴向位置
 * @param cur_cir  运动规划起点周向位置
 * @param _step_axis  步进或扫查运动的轴向距离
 * @param _step_cir   步进或扫查运动的周向距离
 * @param mode        运动模式：1-步进运动；2-扫查运动，3-全覆盖
 */
void MOTION_PLAN::set_motion_range(float cur_aix , float cur_cir,float _step_axis, float _step_cir, int mode){
    // 设置步进或扫查运动的范围
    motion_range.first.x() = cur_aix;
    motion_range.first.y() = cur_cir;
    motion_range.second.x() = cur_aix + _step_axis;
    motion_range.second.y() = cur_cir + _step_cir;

    mode_ = mode;  // 设置运动模式
    motion_en_ = true;  // 使能运动
    scan_positive_en_ = true;
    // 打印运动范围
    if(mode_ == MOTION_MODE::STEP) std::cout << "step motion range: ";
    else if(mode_ == MOTION_MODE::SCAN) std::cout << "scan motion range: ";
    else if(mode_ == MOTION_MODE::FULL_PIPE) std::cout << "full coverage motion range: ";
    std::cout   << "first: (" << motion_range.first.x() << ", " << motion_range.first.y() << "), "
                << "second: (" << motion_range.second.x() << ", " << motion_range.second.y() << ")"
                << std::endl;
}

/**
 * @brief 运动规划期解算函数
 * 
 * @param result_aix    外部传入的接收计算结果的轴向速度指针
 * @param result_cir    外部传入的接收计算结果的周向速度指针
 * @param cur_aix       当前机器人的轴向位置坐标
 * @param cur_cir       当前机器人的周向位置坐标
 * @param printFlag     是否打印调试信息
 * @return steerState 
 */
steerState MOTION_PLAN::motion_plan_(float* result_aix , float* result_cir, 
    float cur_aix , float cur_cir,
    bool printFlag){
    // 计算运动方向
    dir_ = atan2(motion_range.second.y() - motion_range.first.y() , motion_range.second.x() - motion_range.first.x());  
    // 计算步进运动的距离
    trace_distance_ = sqrt(pow(motion_range.second.x() - motion_range.first.x(), 2) + pow(motion_range.second.y() - motion_range.first.y(), 2));  
    float current_distance = sqrt(  pow(cur_aix - motion_range.first.x(), 2) + 
                                    pow(cur_cir  - motion_range.first.y(), 2));
    steerState return_state = steerState::STOP;

    if(motion_en_ == false){
        std::cout<< YELLOW_STRING << BLOD_STRING << "motion plan is not enabled" << RESET_STRING << std::endl;
        *result_aix = 0.0f;
        *result_cir = 0.0f;
        return return_state;
    }

    switch (mode_)
    {
        case MOTION_MODE::STEP:
            if (current_distance >= trace_distance_)
            {
                // 如果当前距离大于等于步进运动的距离，则认为到达终点
                *result_aix = 0.0f;
                *result_cir = 0.0f;
                motion_en_ = false;  // 关闭运动使能
                std::cout << GREEN_STRING << "robot step motion end" << RESET_STRING << std::endl;
            }
            else
            {
                // 如果当前距离小于步进运动的距离，则继续执行步进运动
                *result_aix = default_speed_ * cos(dir_);
                *result_cir = default_speed_ * sin(dir_);
                return_state = steerState::NORMAL;
                if(printFlag) 
                std::cout << BLOD_STRING << GREEN_STRING
                        << "target motion: " << *result_aix << ", " << *result_cir  << "\n"
                        << "current distance: " << current_distance << RESET_STRING << std::endl;
            }
            break;
        case MOTION_MODE::SCAN:
            if(scan_positive_en_){
                // 如果当前距离大于等于扫查运动的距离，则认为到达终点
                if (current_distance >= trace_distance_) scan_positive_en_ = false; // 扫查正向使能为false
                else{
                    // 如果当前距离小于扫查运动的距离，则继续执行扫查运动
                    *result_aix = default_speed_ * cos(dir_);
                    *result_cir = default_speed_ * sin(dir_);
                    return_state = steerState::NORMAL;
                    if(printFlag) 
                    std::cout << BLOD_STRING << GREEN_STRING
                            << "target motion: " << *result_aix << ", " << *result_cir  << "\n"
                            << "current distance: " << current_distance << RESET_STRING << std::endl;
                }
            }else{
                current_distance = sqrt(  pow(cur_aix - motion_range.second.x(), 2) + 
                                        pow(cur_cir  - motion_range.second.y(), 2));
                if (current_distance >= trace_distance_) scan_positive_en_ = true; // 扫查正向使能为false
                else{
                    // 如果当前距离小于扫查运动的距离，则继续执行扫查运动
                    *result_aix = -default_speed_ * cos(dir_);
                    *result_cir = -default_speed_ * sin(dir_);
                    return_state = steerState::NORMAL;
                    if(printFlag) 
                    std::cout << BLOD_STRING << GREEN_STRING
                            << "target motion: " << *result_aix << ", " << *result_cir  << "\n"
                            << "current distance: " << current_distance << RESET_STRING << std::endl;
                }
            }
            break;
        default:
            break;
    }

    if(printFlag){
        std::cout << "motion range: " 
            << "first: (" << motion_range.first.x() << ", " << motion_range.first.y() << "), "
            << "second: (" << motion_range.second.x() << ", " << motion_range.second.y() << ")" 
            << "\n"
            << "motion direction: " << dir_
            << "\t"
            << "motion trace distance: " << trace_distance_
            << std::endl;
    }

}

void MOTION_PLAN::reset_motion(){
    motion_en_ = false;
    mode_ = 0;
    motion_range.first.x() = 0.0f;
    motion_range.first.y() = 0.0f;
    motion_range.second.x() = 0.0f;
    motion_range.second.y() = 0.0f;
}

