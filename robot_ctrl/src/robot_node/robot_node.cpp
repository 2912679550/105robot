#include "robot_node.hpp"
#include "robot_params.hpp"

// ! ========================== main robot ===========================
// ! ========================== main robot ===========================

MAIN_ROBOT::MAIN_ROBOT(ros::NodeHandle* nh_ ){
    if(nh_ == nullptr){
        nh_ = new ros::NodeHandle();
        std::cout<< YELLOW_STRING << BLOD_STRING << "nh_ is nullptr" << RESET_STRING << std::endl;
    }
    front_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_F, STM_ROBOT_VAL_F, nh_);
    back_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_B, STM_ROBOT_VAL_B, nh_);
    push_ctrl_ = new PUSH_CTRL(PUSH_CMD , PUSH_VAL, nh_);
    front_side_->create_imu(IMU_FRONT , IMU_ID::FRONT);  // 创建前侧IMU
    back_side_->create_imu(IMU_BACK , IMU_ID::BACK);   // 创建后侧IMU
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

void MAIN_ROBOT::motion_cmd_callback(const TCP_ROBOT_CMD_CPTR &msg){
    if(msg != nullptr)
    {
        // 处理接收到的数据，整合成一个字符串容器
        std::string mode = msg->cmdType;
        std::cout<< BLUE_STRING << BLINK_STRING
            << "receive command: " << mode
            << RESET_STRING << std::endl;
        if(mode == ROBOT_STOP){
            front_side_->set_steer(steerState::STOP);
            back_side_->set_steer(steerState::STOP);
        }
        else if(mode == ROBOT_CALI){
            front_side_->set_steer(steerState::RESET);
            back_side_->set_steer(steerState::RESET);
        }
        else if(mode == ROBOT_MOTION){
            front_side_->set_steer(steerState::NORMAL , msg->v_axi , msg->v_cir);
            back_side_->set_steer(steerState::NORMAL , msg->v_axi , msg->v_cir);
        }
        else if(mode == ROBOT_TIGHT_EN){
            front_side_->set_tight(49.0f);
            back_side_-> set_tight(49.0f);
            // front_side_->set_tight(47.0f);
            // back_side_-> set_tight(47.0f);
        }
        else if(mode == ROBOT_TIGHT_DIS){
            front_side_->set_tight(false);
            back_side_->set_tight(false);
        }
        else if(mode == ROBOT_ANGLE){
            front_side_->set_angle(msg->angle_front);
            back_side_->set_angle(msg->angle_back);
        }
        else if(mode == ROBOT_LOSS_F){
            front_side_->set_tight(false);
            front_side_->release_quat();  // 释放前侧IMU的四元数    
            back_side_->fix_quat();  // 后侧单边锁住
        }
        else if(mode == ROBOT_LOSS_B){
            back_side_->set_tight(false);
            back_side_->release_quat();  // 释放后侧IMU的四元数
            front_side_->fix_quat();  // 前侧单边锁住
        }
        else if(mode == ROBOT_OPEN){
            push_ctrl_->set_cmd(70.0f , 70.0f ,20.0f);
        }
        else if(mode == ROBOT_CLOSE){
            push_ctrl_->set_cmd(20.0f , 20.0f , 20.0f);
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
    // 如果只有一侧夹紧状态为false，则将其自动失能
    if(front_side_->tarTightFlag_ == false && back_side_-> tarTightFlag_ == true) front_side_ -> set_steer(steerState::STOP);
    if(back_side_->tarTightFlag_ == false && front_side_-> tarTightFlag_ == true) back_side_ -> set_steer(steerState::STOP);
    
    front_side_->single_side_ctrl();  // 前侧单侧控制逻辑
    back_side_->single_side_ctrl();   // 后侧单侧控制逻辑

    // todo 里程计逻辑处理
    bool odomPrintFlag = false;  // 是否打印里程计数据
    // * 根据夹紧状态自动控制里程计启停
    // 总体概括为：夹紧状态为false，则暂停里程计；
    // 夹紧状态为true，则判断是否已经夹紧了超过1s，如果超过1s，则开始里程计；否则暂停里程计
    if(front_side_ -> cur_tightFlag_ == false){
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
    if(back_side_ -> cur_tightFlag_ == false){
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
        robot_axis_odom_ = (front_side_->odom_handler_->odom_axis + back_side_->odom_handler_->odom_axis) / 2.0f;
        robot_cir_odom_ = (front_side_->odom_handler_->odom_cir + back_side_->odom_handler_->odom_cir) / 2.0f;
        odomPrintFlag = true;  // 设置打印标志位为true
    }
    else if(front_odom_en_ && (!back_odom_en_)){
        // 如果只有前侧里程计使能，则取前侧里程计值，并使用前侧里程计刷新后侧里程计
        robot_axis_odom_ = front_side_->odom_handler_->odom_axis;
        robot_cir_odom_ = front_side_->odom_handler_->odom_cir;
        back_side_->odom_handler_->set_cur_val(robot_axis_odom_, robot_cir_odom_);  // 刷新后侧里程计
        odomPrintFlag = true;  // 设置打印标志位为true
    }
    else if((!front_odom_en_) && back_odom_en_){
        // 如果只有后侧里程计使能，则取后侧里程计值，并使用后侧里程计刷新前侧里程计
        robot_axis_odom_ = back_side_->odom_handler_->odom_axis;
        robot_cir_odom_ = back_side_->odom_handler_->odom_cir;
        front_side_->odom_handler_->set_cur_val(robot_axis_odom_, robot_cir_odom_);  // 刷新前侧里程计
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
    // 发布控制指令
    pubCmd();


    // 发布手柄回传数据
    // ROBOT_TCP_VAL_TYPE tcp_val;

    // tcp_pub_.publish(tcp_val);
}



