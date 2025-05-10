#include "robot_node.hpp"
#include "robot_params.hpp"

// ! ========================== single side ctrl ===========================
// ! ========================== single side ctrl ===========================

SINGLE_SIDE_CTRL::SINGLE_SIDE_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh){
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
    }else{
        nh_ = nh;
    }
    cmd_pub_ = nh_->advertise<ROBOT_STM_CMD_TYPE>(cmd_topic, 1);
    val_sub_ = nh_->subscribe(val_topic, 1, &SINGLE_SIDE_CTRL::val_callback, this);
    std::cout<< GREEN_STRING << BLOD_STRING << "single side ctrl node start\n" 
            << " cmd_topic: " << cmd_topic << "\n"
            << " val_topic: " << val_topic
            << RESET_STRING << std::endl;

    // 初始化控制指令
    for (int i = 0; i < 3;i++){
        cmd_data_.dir_steer_state[i] = steerState::STOP;  // 舵轮当前的工作状态
        cmd_data_.dir_steer_dir[i] = 0.5 * PI;  // 舵轮舵向的角度
        cmd_data_.dir_steer_vel[i] = 0.0f;  // 舵轮舵向的速度
        cmd_data_.dir_spring_length = 1.0f;  // 松开
    }
}

SINGLE_SIDE_CTRL::~SINGLE_SIDE_CTRL(){
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}

void SINGLE_SIDE_CTRL::pub_cmd(){
    cmd_pub_.publish(cmd_data_);
}

void SINGLE_SIDE_CTRL::val_callback(const STM_ROBOT_VAL_CPTR &msg){
    if(msg != nullptr)
    val_data_ = *msg;
}

void SINGLE_SIDE_CTRL::set_tight(bool tightFlag){
    cmd_data_.dir_spring_length = tightFlag ? 20.0f : 1.0f;
}

void SINGLE_SIDE_CTRL::set_angle(float angle){
    cmd_data_.dir_arm_angle[0] = 180.0 - angle;
    cmd_data_.dir_arm_angle[1] = 180.0 - angle;
}

void SINGLE_SIDE_CTRL::set_steer(steerState stateIn , float v_aix , float v_cir){
    for (int i = 0; i < 3;i++){
        // v_aix 为轴向速度，v_cir为周向速度，周向速度的正方向对应于舵轮舵向的0弧度处
        if(stateIn == steerState::RESET || stateIn == steerState::STOP){
            cmd_data_.dir_steer_state[i] = stateIn;
            // 配置但不使用
            cmd_data_.dir_steer_dir[i] = 0.5 * PI;
            cmd_data_.dir_steer_vel[i] = 0.0f;
        }
        else if(stateIn = steerState::NORMAL){
            // 
            cmd_data_.dir_steer_state[i] = stateIn;
            float vel_total = sqrt(v_aix * v_aix + v_cir * v_cir);  // 速度的数值大小
            float vel_dir = atan2(v_aix , v_cir);  // 速度的方向 , 这里的角度范围为[-PI , PI]
            // 将角度范围调整到（0,PI]，并为此修正速度大小
            if(vel_dir <= 0.0f){
                vel_dir += 1.0f * PI;
                vel_total = -vel_total;  
            }
            cmd_data_.dir_steer_dir[i] = vel_dir;  // 舵轮舵向的角度
            cmd_data_.dir_steer_vel[i] = vel_total;  // 舵轮舵向的速度
        }
    }
}

// ! ========================== push ctrl ===========================
// ! ========================== push ctrl ===========================
PUSH_CTRL::PUSH_CTRL(std::string cmd_topic , std::string val_topic , ros::NodeHandle* nh){
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
    }else{
        nh_ = nh;
    }
    cmd_pub_ = nh_->advertise<PUSH_CMD_TYPE>(cmd_topic, 1);
    val_sub_ = nh_->subscribe(val_topic, 1, &PUSH_CTRL::val_callback, this);
    std::cout<< GREEN_STRING << BLOD_STRING << "push ctrl node start\n" 
            << " cmd_topic: " << cmd_topic << "\n"
            << " val_topic: " << val_topic
            << RESET_STRING << std::endl;
    
    // 初始化控制指令
    cmd_data_.tar_length_f = 20.0f;  // 前推杆的目标长度
    cmd_data_.tar_length_b = 20.0f;  // 后推杆的目标长度
    cmd_data_.tar_length_m = 15.0f;  // 中推杆的目标长度
}

PUSH_CTRL::~PUSH_CTRL(){
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}

void PUSH_CTRL::pub_cmd(){
    cmd_pub_.publish(cmd_data_);
}

void PUSH_CTRL::set_cmd(float tar_length_f , float tar_length_b , float tar_length_m){
    cmd_data_.tar_length_f = tar_length_f;
    cmd_data_.tar_length_b = tar_length_b;
    cmd_data_.tar_length_m = tar_length_m;
}

void PUSH_CTRL::val_callback(const PUSH_VAL_CPTR &msg){
    if(msg != nullptr)
    val_data_ = *msg;
}

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
            front_side_->set_tight(true);
            back_side_->set_tight(true);
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
        }
        else if(mode == ROBOT_LOSS_B){
            back_side_->set_tight(false);
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

