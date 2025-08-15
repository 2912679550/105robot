#include "board_ctrl.hpp"
#include "robot_params.hpp"
#include "tf/transform_datatypes.h"

// ! ========================== micro odom ctrl ===========================
// ! ========================== micro odom ctrl ===========================
MICRO_ODOM::MICRO_ODOM(float* coeff){
    // 初始化里程计数据
    for (int i = 0; i < 3; i++){
        pre_position_[i] = Eigen::Vector2f(0.0f, 0.0f);  // 初始化为零
        cur_position_[i] = Eigen::Vector2f(0.0f, 0.0f);  // 初始化为零
    }
    if (coeff != nullptr) {
        for (int i = 0; i < 3; i++) {
            this->coeff[i] = coeff[i];  // 如果传入了系数，则使用传入的系数
        }
    } else {
        this->coeff[0] = 1.0f;  // 默认系数
        this->coeff[1] = 1.0f;  // 默认系数
        this->coeff[2] = 1.0f;  // 默认系数
    }
}

MICRO_ODOM::~MICRO_ODOM(){
    // 清理里程计数据
    for (int i = 0; i < 3; i++){
        pre_position_[i].setZero();  // 清零
        cur_position_[i].setZero();  // 清零
    }
}

void MICRO_ODOM::reset(){
    // 重置里程计标志
    resetFlag = true;
}

void MICRO_ODOM::set_cur_val(float* axis, float* cir){
    memcpy(odom_axis, axis, 3 * sizeof(float));  // 复制轴向里程计数据
    memcpy(odom_cir, cir, 3 * sizeof(float));    // 复制周向里程计数据
}

void MICRO_ODOM::update(STM_ROBOT_VAL_TYPE *val_data, bool printFlag ){
    if(resetFlag == true){
        memset(odom_axis, 0, sizeof(odom_axis));
        memset(odom_cir, 0, sizeof(odom_cir));
        odom_axis_avg = 0.0f;  // 重置平均轴向里程计
        odom_cir_avg = 0.0f;   // 重置平均周向里程计
        resetFlag = false;  // 重置标志位
        for (int i = 0; i < 3; i++){
            pre_position_[i].setZero();  // 清零上一次的位置
            cur_position_[i].setZero();   // 清零当前的位置
        }
    }
    else{
        Eigen::Vector2f delta[3];
        for (int i = 0; i < 3; i++)
        {
            pre_position_[i] = cur_position_[i];  // 保存上一次的位置
            cur_position_[i] = Eigen::Vector2f(val_data->odom_axis[i], val_data->odom_cir[i]);  // 更新当前的位置
            delta[i] = cur_position_[i] - pre_position_[i];  // 计算位置增量
            delta[i] *= coeff[i];  // 应用系数调整增量
            if(startOdom == true){
                // 如果开启了里程计，则进行累计更新
                odom_axis[i] += delta[i].x();  // 更新轴向里程计
                odom_cir[i] += delta[i].y();    // 更新周向里程计
            }
        }
        odom_axis_avg = (odom_axis[0] + odom_axis[1] + odom_axis[2]) / 3.0f;  // 计算平均轴向里程计
        odom_cir_avg = (odom_cir[0] + odom_cir[1] + odom_cir[2]) / 3.0f;  // 计算平均周向里程计
    }
}

// ! ========================== single side ctrl ===========================
// ! ========================== single side ctrl ===========================

SINGLE_SIDE_CTRL::SINGLE_SIDE_CTRL(std::string cmd_topic, std::string val_topic, ros::NodeHandle *nh)
{
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
        cmd_data_.dir_spring_length = 60.0f;  // 松开
    }
    odom_handler_ = new MICRO_ODOM();  // 创建里程计处理类实例
    tight_timer_ = new MYTIMER();  // 创建定时器处理类实例
    tight_timer_->reset();
    odom_handler_->reset(); // 重置里程计数据
    set_dia(340.0f);  // 设置舵轮的直径，单位为mm
}

SINGLE_SIDE_CTRL::~SINGLE_SIDE_CTRL(){
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}


void SINGLE_SIDE_CTRL::single_side_ctrl(){
    // 单侧控制逻辑
    // pre_tightFlag_ = cur_tightFlag_;  // 保存上一次的夹紧状态
    // if(tarTightFlag_ == true){
    //     // 如果当前目标状态为夹紧，则开始判断是否到达夹紧阈值
    //     if(cmd_data_.dir_spring_length -  val_data_.cur_spring_length[0] > -0.5f
    //         && cmd_data_.dir_spring_length -  val_data_.cur_spring_length[1] > -0.5f){
    //         // 如果夹紧差长度小于0.5f，则认为已经夹紧
    //         cur_tightFlag_ = true;  // 更新当前夹紧状态
    //         // 如果上一次夹紧状态为未夹紧，则重置定时器
    //         if(pre_tightFlag_ == false) tight_timer_->reset();
    //     }else{
    //         cur_tightFlag_ = false;  // 否则认为未夹紧
    //         tight_timer_->reset();  // 重置定时器
    //     }
    // }else{
    //     // 如果当前目标状态为松开，则认为未夹紧
    //     cur_tightFlag_ = false;
    //     tight_timer_->reset();  // 重置定时器
    // }
}

void SINGLE_SIDE_CTRL::pub_cmd(){
    // if(steer_state_ == steerState::NORMAL) {
    //     // 如果当前是正常状态，迭代计算舵轮状态，可能需要姿态矫正的参与
    //     set_steer(steerState::NORMAL , tar_v_aix_ , tar_v_cir_);  
    // }
    cmd_pub_.publish(cmd_data_);
}

void SINGLE_SIDE_CTRL::val_callback(const STM_ROBOT_VAL_CPTR &msg){
    if(msg != nullptr)
    val_data_ = *msg;
    // 里程计会自动接收数据并更新，但会有标志位控制其输出是否会累加
    if(odom_handler_ != nullptr) odom_handler_->update(&val_data_, false);  // 更新里程计数据，不打印
}

void SINGLE_SIDE_CTRL::set_tight(bool tightFlag){
    cmd_data_.dir_spring_length = tightFlag ? 20.0f : 1.0f;
    if(cmd_data_.dir_spring_length < 10.0f){
        tarTightFlag_ = false;  // 如果长度小于10.0f，认为是松开状态
    }else{
        tarTightFlag_ = true;  // 否则认为是夹紧状态
    }
}

void SINGLE_SIDE_CTRL::set_tight(float length){
    cmd_data_.dir_spring_length = length;
    if(cmd_data_.dir_spring_length < 10.0f || cmd_data_.dir_spring_length > 59.0f){
        tarTightFlag_ = false;  // 如果长度小于10.0f，认为是松开状态
    }else{
        tarTightFlag_ = true;  // 否则认为是夹紧状态
    }
}

void SINGLE_SIDE_CTRL::set_angle(float angle){
    cmd_data_.dir_arm_angle[0] = 180.0 - angle;
    cmd_data_.dir_arm_angle[1] = 180.0 - angle;
}

void SINGLE_SIDE_CTRL::set_dia(float dia){
    // 这里输入的dia单位应为mm，限制为100mm 到370mm
    if(dia < 100.0f || dia > 370.0f){
        std::cout << RED_STRING << "dia out of range: " << dia << RESET_STRING << std::endl;
        return;
    }
    double target_angle =   pow(double(dia/100.0f) , 6) * dia2mechAngelCoeff[0] +
                            pow(double(dia/100.0f) , 5) * dia2mechAngelCoeff[1] +
                            pow(double(dia/100.0f) , 4) * dia2mechAngelCoeff[2] +
                            pow(double(dia/100.0f) , 3) * dia2mechAngelCoeff[3] +
                            pow(double(dia/100.0f) , 2) * dia2mechAngelCoeff[4] +
                            pow(double(dia/100.0f) , 1) * dia2mechAngelCoeff[5] +
                            dia2mechAngelCoeff[6];
    // 设置目标夹角
    std::cout   << GREEN_STRING << "set dia: " << dia 
                << " target angle: " << target_angle << RESET_STRING << std::endl;
    set_angle(target_angle);
}

/**
 * @brief 用于配置单边共计三个舵轮的工作状态与指令
 * 
 * @param stateIn   期望舵轮工作状态 ，与底层32对应
 * @param v_aix     理论期望轴向速度
 * @param v_cir     理论期望周向速度
 * @param pid_out_p     来自外部在pitch方向的矫正
 * @param pid_out_y     来自外部在yaw方向的矫正
 */
void SINGLE_SIDE_CTRL::set_steer(steerState stateIn , float v_aix , float v_cir , float pid_out_p , float pid_out_y){
    steer_state_ = stateIn;  // 更新当前舵轮的工作状态
    tar_v_aix_ = v_aix;  // 更新目标轴向速度
    tar_v_cir_ = v_cir;  // 更新目标周向速度

    // v_aix 为轴向速度，v_cir为周向速度，周向速度的正方向对应于舵轮舵向的0弧度处
    if(stateIn == steerState::RESET || stateIn == steerState::STOP){
        for (int i = 0; i < 3;i++){
        cmd_data_.dir_steer_state[i] = stateIn;
        // 配置但不使用
        cmd_data_.dir_steer_dir[i] = 0.5 * PI;
        cmd_data_.dir_steer_vel[i] = 0.0f;
        }
    }
    else if (stateIn = steerState::NORMAL)
    {
        for (int i = 0; i < 3; i++)
        {
            cmd_data_.dir_steer_state[i] = stateIn;
            if (i == 2)
                v_aix = tar_v_aix_ + pid_out_p; // 如果是主动驱动轮，误差为正时应该增大轴向速度
            else{
                // i = 0 为 左侧轮子，i = 1 为右侧轮子
                v_aix = tar_v_aix_; //  - pid_out_p; // 辅助驱动轮，误差为正时应该减小轴向速度
                if (i == 0) v_aix = v_aix + pid_out_y * (-1.0f);    // 左侧轮子需要减速，右侧轮子需要提速
                if (i == 1) v_aix = v_aix + pid_out_y * ( 1.0f);    // 左侧轮子需要减速，右侧轮子需要提速
            }
            float vel_total = sqrt(v_aix * v_aix + v_cir * v_cir); // 速度的数值大小
            float vel_dir = atan2(v_aix, v_cir);                   // 速度的方向 , 这里的角度范围为[-PI , PI]
            // 将角度范围调整到（0,PI]，并为此修正速度大小
            if (vel_dir <= 0.0f)
            {
                vel_dir += 1.0f * PI;
                vel_total = -vel_total;
            }

            if(i < 2 && pipdiffFlag_ == true){
                // 开启了弯道差速
                vel_total = vel_total * (300.0 - 0.5 * pipe_r_) / (300.0 + pipe_r_); // 辅助轮的速度需要根据管道半径进行调整
            }

            cmd_data_.dir_steer_dir[i] = vel_dir;   // 舵轮舵向的角度
            cmd_data_.dir_steer_vel[i] = vel_total; // 舵轮舵向的速度
        }
    }
}
/**
 * @brief 拆分开独立控制主轮与辅助轮的速度接口，主要用于自动进弯时的速度配置
 * 
 * @param main_speed    主动轮的速度数组指针，接收一个float[2]的数组，分别对应轴向与周向速度
 * @param assist_speed  辅助轮的速度数组指针，接收一个float[2]的数组，分别对应轴向与周向速度
 */
void SINGLE_SIDE_CTRL::set_main_assist_speed_(float* main_speed , float* assist_speed){
    if(main_speed != nullptr && assist_speed != nullptr){
        for (int i = 0; i < 3; i++)
        {
            cmd_data_.dir_steer_state[i] = steerState::NORMAL;  // 设置舵轮工作状态为正常
            float vel_total = 0.0; // 速度的数值大小
            float vel_dir = 0.0;                   // 速度的方向 , 这里的角度范围为[-PI , PI]
            if( i ==2){
                vel_total = sqrt(main_speed[0] * main_speed[0] + main_speed[1] * main_speed[1]); // 主动轮的速度
                vel_dir = atan2(main_speed[0], main_speed[1]); // 主动轮的速度方向
            }else{
                vel_total = sqrt(assist_speed[0] * assist_speed[0] + assist_speed[1] * assist_speed[1]); // 辅助轮的速度
                vel_dir = atan2(assist_speed[0], assist_speed[1]); // 辅助轮的速度方向
            }
            cmd_data_.dir_steer_dir[i] = vel_dir;   // 舵轮舵向的角度
            cmd_data_.dir_steer_vel[i] = vel_total; // 舵轮
        }
    }
}

void SINGLE_SIDE_CTRL::pipe_sped_diff(bool pipdiffFlag , float pipe_r){
    pipdiffFlag_ = pipdiffFlag;
    pipe_r_ = pipe_r;
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
    cmd_data_.tar_length_f = 25.0f;  // 前推杆的目标长度
    cmd_data_.tar_length_b = 25.0f;  // 后推杆的目标长度
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
/**
 * @brief 用于直接配置机器人推杆的长度，对应的是模型意义上的长度，主要用于接收求解器解算的理论长度值，会在内部换算成推杆的期望长度
 * 
 * @param targetLength 目标的模型长度， unit:mm
 * @param isFront  是否为前侧推杆，true为前侧，false为后侧
 */
void PUSH_CTRL::set_body_length(float targetLength, bool isFront){
    static float deltaLength = body_angle_length_basline - body_angle_push_baseline; // 模型期望长度 - deltaLength = 推杆控制长度
    float push_length = targetLength - deltaLength;
    push_length = set_range<float>(push_length, (float*)push_out_length); // 限制推杆伸出长度在合理范围内
    if(isFront) cmd_data_.tar_length_f = push_length;
    else cmd_data_.tar_length_b = push_length;
}

void PUSH_CTRL::set_body_length(float length_f, float length_b){
    length_f = set_range<float>(length_f, (float*)push_out_length); // 限制前推杆伸出长度在合理范围内
    length_b = set_range<float>(length_b, (float*)push_out_length); // 限制后推杆伸出长度在合理范围内
    cmd_data_.tar_length_f = length_f;
    cmd_data_.tar_length_b = length_b;
}

void PUSH_CTRL::set_body_angle(float angle){
    angle = angle > body_angle_range[1] ? body_angle_range[1] : angle;  // 限制角度不超过基准值
    angle = angle < body_angle_range[0] ? body_angle_range[0] : angle;  // 限制角度不低于基准值
    // ! 临时测试用 0731
    if(angle < -19.0f)
        angle = -30.0;
    // ! 临时测试end
    float delta_angel = angle / 2.0f;
    float angle_BCE = delta_angel + body_angle_baseline; // 此时单位为度   
    // 余弦定理计算推杆总长度
    // 56.73 
    float tar_length = sqrt(body_length_BC * body_length_BC + body_length_CE * body_length_CE - 
                            2 * body_length_BC * body_length_CE * cos(angle_BCE * PI / 180.0f));
    // 计算推杆期望长度
    // 相当于将推杆长度的变化量作用于弯折角为0度时的推杆长度上
    cmd_data_.tar_length_f = tar_length - body_angle_length_basline + body_angle_push_baseline; 
    cmd_data_.tar_length_b = cmd_data_.tar_length_f;  // 前后推杆长度相同
}
