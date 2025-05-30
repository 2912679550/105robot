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
            front_side_->set_tight(47.0f);
            back_side_->set_tight(47.0f);
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
    // 发布控制指令
    pubCmd();


    // 发布手柄回传数据
    // ROBOT_TCP_VAL_TYPE tcp_val;

    // tcp_pub_.publish(tcp_val);
}



