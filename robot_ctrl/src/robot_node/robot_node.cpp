#include "robot_node.hpp"
#include "robot_params.hpp"

// ! ========================== main robot ===========================
// ! ========================== main robot ===========================

float odomValueCoeff_f[3] = {1.040268, 1.05457, 1.016019};
float odomValueCoeff_b[3] = {1.048881, 1.063183, 0.994876}; // 里程计值的缩放系数

MAIN_ROBOT::MAIN_ROBOT(ros::NodeHandle* nh_ ){
    if(nh_ == nullptr){
        nh_ = new ros::NodeHandle();
        std::cout<< YELLOW_STRING << BLOD_STRING << "nh_ is nullptr" << RESET_STRING << std::endl;
    }
    front_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_F, STM_ROBOT_VAL_F, nh_);
    back_side_ = new SINGLE_SIDE_CTRL(ROBOT_STM_CMD_B, STM_ROBOT_VAL_B, nh_);
    push_ctrl_ = new PUSH_CTRL(PUSH_CMD , PUSH_VAL, nh_);

    front_side_->odom_handler_ = new MICRO_ODOM(odomValueCoeff_f);  // 前侧里程计处理类
    back_side_->odom_handler_ = new MICRO_ODOM(odomValueCoeff_b);

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

        // * 基本运动与功能控制
        if(mode == ROBOT_STOP){
            front_side_->set_steer(steerState::STOP);
            back_side_->set_steer(steerState::STOP);
            step_moiton_en_ = false;
            scan_motion_en_ = false;
        }
        else if(mode == ROBOT_CALI){
            front_side_->set_steer(steerState::RESET);
            back_side_->set_steer(steerState::RESET);
        }
        else if(mode == ROBOT_MOTION){
            front_side_->set_steer(steerState::NORMAL , msg->v_axi , msg->v_cir);
            back_side_->set_steer(steerState::NORMAL , msg->v_axi , msg->v_cir);
            // 重置步进与扫查运动的使能标志位
            step_moiton_en_ = false;
            scan_motion_en_ = false;
        }
        else if(mode == ROBOT_STEP){
            // 步进运动
            // if(step_moiton_en_ == false){  // 如果步进运动使能为false，则开始步进运动
            step_moiton_en_ = true;  // 设置步进运动使能为true
            scan_motion_en_ = false;  // 扫查运动使能为false
            // 设置运动范围
            set_motion_range(msg->v_axi, msg->v_cir);
            // 打印运动范围
            std::cout << "step motion range: " 
                      << "first: (" << motion_range.first.x() << ", " << motion_range.first.y() << "), "
                      << "second: (" << motion_range.second.x() << ", " << motion_range.second.y() << ")" 
                      << std::endl;
        }
        else if(mode == ROBOT_SCAN){
            // 扫查运动 
            step_moiton_en_ = false;  // 步进运动使能为false
            scan_motion_en_ = true;
            scan_positive_en_ = true;  // 扫查正向使能为true
            // 设置运动范围
            set_motion_range(msg->v_axi, msg->v_cir);
            // 打印运动范围
            std::cout << "scan motion range: " 
                      << "first: (" << motion_range.first.x() << ", " << motion_range.first.y() << "), "
                      << "second: (" << motion_range.second.x() << ", " << motion_range.second.y() << ")" 
                      << std::endl;
        }

        // * 夹紧控制
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
        else if(mode == ROBOT_TIGHT_F){
            front_side_->set_tight(47.0f);  // 前侧夹紧
        }
        else if(mode == ROBOT_LOSS_F){
            front_side_->set_tight(false);
            front_side_->release_quat();  // 释放前侧IMU的四元数    
            if( front_side_->tarTightFlag_ == false ) back_side_ ->fix_quat();  // 如果此时前侧臂为松开状态，则后侧单边锁住
        }
        else if(mode == ROBOT_TIGHT_B){ 
            back_side_->set_tight(47.0f);  // 后侧夹紧
        }
        else if(mode == ROBOT_LOSS_B){
            back_side_->set_tight(false);
            back_side_->release_quat();  // 释放后侧IMU的四元数
            if( back_side_->tarTightFlag_  == false ) front_side_->fix_quat();  // 前侧单边锁住
        }

        // * 变形控制
        else if(mode == ROBOT_DIA){
            front_side_->set_dia(msg->dia_front);
            back_side_->set_dia(msg->dia_back);
        }
        else if(mode == ROBOT_BODY_ANGLE){
            push_ctrl_->set_body_angle(msg->robot_kink_angle);
        }
        else{
            std::cout<< RED_STRING << "robot node receive unknown command" << RESET_STRING << std::endl;
        }
        
        // * 旧指令
        // else if(mode == ROBOT_ANGLE){
        //     front_side_->set_angle(msg->angle_front);
        //     back_side_->set_angle(msg->angle_back);
        // }
        // else if(mode == ROBOT_OPEN){
        //     push_ctrl_->set_cmd(70.0f , 70.0f ,20.0f);
        // }
        // else if(mode == ROBOT_CLOSE){
        //     push_ctrl_->set_cmd(20.0f , 20.0f , 20.0f);
        // }
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


    // * 步进与扫查运动控制
    float speed = 0.02f;  // 步进或扫查运动的速度
    float dir = 0.0f;
    float trace_distance = 0.0f;  // 步进或扫查运动的距离
    if(step_moiton_en_ || scan_motion_en_){
        // 使能了步进或扫查运动，则现在这里根据运动范围来解算速度方向
        dir = atan2(motion_range.second.y() - motion_range.first.y() , motion_range.second.x() - motion_range.first.x());  // 计算运动方向
        trace_distance = sqrt(pow(motion_range.second.x() - motion_range.first.x(), 2) + pow(motion_range.second.y() - motion_range.first.y(), 2));  // 计算步进运动的距离
        std::cout << "motion range: " 
                    << "first: (" << motion_range.first.x() << ", " << motion_range.first.y() << "), "
                    << "second: (" << motion_range.second.x() << ", " << motion_range.second.y() << ")" 
                    << "\n"
                    << "motion direction: " << dir
                    << "\t"
                    << "motion trace distance: " << trace_distance
                    << std::endl;
    }
    if (step_moiton_en_)
    {
        // 步进运动控制
        // 使用当前距离到起点的距离来判断是否到达步进运动终点（步进运动使用大于阈值判断）
        float current_distance = sqrt(  pow(robot_axis_odom_ - motion_range.first.x(), 2) + 
                                        pow(robot_cir_odom_  - motion_range.first.y(), 2));
        if(current_distance >= trace_distance){
            // 如果当前距离大于等于步进运动的距离，则认为到达终点
            step_moiton_en_ = false;  // 步进运动使能为false
            scan_motion_en_ = false;  // 扫查运动使能为false
            front_side_->set_steer(steerState::STOP);  // 前侧舵轮停止
            back_side_->set_steer(steerState::STOP);   // 后侧舵轮停止
            std::cout<< GREEN_STRING << "robot step motion end" << RESET_STRING << std::endl;
        }else{
            // 如果当前距离小于步进运动的距离，则继续执行步进运动
            front_side_->set_steer(steerState::NORMAL, speed * cos(dir), speed * sin(dir));  // 前侧舵轮运动
            back_side_->set_steer(steerState::NORMAL, speed * cos(dir), speed * sin(dir));   // 后侧舵轮运动
            std::cout<< BLOD_STRING
                    << GREEN_STRING
                    << "target motion: "
                    << speed * cos(dir) << ", " << speed * sin(dir)
                    << "\n"
                    <<"current distance: "<< current_distance
                    << RESET_STRING << std::endl;
        }
    }
    else if (scan_motion_en_)
    {
        // 扫查运动控制
        if (scan_positive_en_)
        {
            // 扫查正向使能
            // 使用当前距离到起点的距离来判断是否到达扫查运动终点（扫查运动使用小于阈值判断）
            float current_distance = sqrt(  pow(robot_axis_odom_ - motion_range.first.x(), 2) +
                                            pow(robot_cir_odom_ - motion_range.first.y(), 2));
            if (current_distance >= trace_distance)
            {
                // 如果当前距离小于等于扫查运动的距离，则认为到达终点
                scan_positive_en_ = false; // 扫查正向使能为false
            }
            else
            {
                // 如果当前距离大于扫查运动的距离，则继续执行扫查运动
                front_side_->set_steer(steerState::NORMAL, speed * cos(dir), speed * sin(dir)); // 前侧舵轮运动
                back_side_->set_steer(steerState::NORMAL, speed * cos(dir), speed * sin(dir));  // 后侧舵轮运动
            }
        }else{
            // 扫查反向使能
            // 使用当前距离到终点的距离来判断是否到达扫查运动终点（扫查运动使用小于阈值判断）
            float current_distance = sqrt(  pow(robot_axis_odom_ - motion_range.second.x(), 2) + 
                                            pow(robot_cir_odom_ - motion_range.second.y(), 2));  
            if (current_distance >= trace_distance)
            {
                scan_positive_en_ = true;  // 扫查运动使能为true
            }else{
                // 如果当前距离小于扫查运动的距离，则继续执行扫查运动
                front_side_->set_steer(steerState::NORMAL, -speed * cos(dir), -speed * sin(dir));
                back_side_->set_steer(steerState::NORMAL, -speed * cos(dir), -speed * sin(dir));
            }
        }
    }
    // * 步进与扫查控制结束

    odom_handler(printFlag); // 处理里程计数据
    // 发布控制指令
    pubCmd();
    // 发布手柄回传数据
    // ROBOT_TCP_VAL_TYPE tcp_val;

    // tcp_pub_.publish(tcp_val);
}

void MAIN_ROBOT::odom_handler(bool printFlag){
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
}

void MAIN_ROBOT::set_motion_range(float _step_axis, float _step_cir){
    // 设置步进或扫查运动的范围
    motion_range.first.x() = robot_axis_odom_;
    motion_range.first.y() = robot_cir_odom_;
    motion_range.second.x() = robot_axis_odom_ + _step_axis;
    motion_range.second.y() = robot_cir_odom_ + _step_cir;
    
    // std::cout<< YELLOW_STRING << BLOD_STRING << "set motion range: " 
    //     << motion_range.first.x() << " , " << motion_range.first.y() 
    //     << " -> " << motion_range.second.x() << " , " << motion_range.second.y()
    //     << RESET_STRING << std::endl;
}
