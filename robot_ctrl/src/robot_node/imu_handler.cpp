#include "imu_handler.hpp"
#include <tf/transform_datatypes.h>

IMU_HANDLER::IMU_HANDLER(std::string imu_topic , ros::NodeHandle *nh)
{
    if(nh == nullptr){
        nh_ = new ros::NodeHandle();
    }else{
        nh_ = nh;
    }
    imu_sub_ = nh_->subscribe(imu_topic, 1, &IMU_HANDLER::imu_callback, this);
    std::cout << GREEN_STRING << BLOD_STRING << "imu topic: " << imu_topic << RESET_STRING << std::endl;
}

IMU_HANDLER::~IMU_HANDLER()
{
    if(nh_ != nullptr){
        delete nh_;
        nh_ = nullptr;
    }
}

void IMU_HANDLER::imu_callback(const IMU_CPTR &msg){
    // 这里接收到的是四元数格式的sensor_msgs::Imu数据，先转换为欧拉角格式的IMU_POSE数据
    if(msg != nullptr){
        tf::Quaternion quat;
        quat.setX(msg->orientation.x);
        quat.setY(msg->orientation.y);
        quat.setZ(msg->orientation.z);
        quat.setW(msg->orientation.w);
        // 转换为RPY
        tf::Matrix3x3 tempRota = tf::Matrix3x3(quat);
        // 将IMU数据转换为机器人坐标系下的姿态
        if( imu_robot_matrix != nullptr){
            tempRota = *imu_robot_matrix * tempRota;  // 将IMU数据转换为机器人坐标系下的姿态
        }
        tempRota.getRPY(ground_truth.roll, ground_truth.pitch, ground_truth.yaw);  // 获取欧拉角
        // 此时的单位为弧度，转化为角度
        ground_truth.roll = ground_truth.roll * 180.0 / M_PI;
        ground_truth.pitch = ground_truth.pitch * 180.0 / M_PI;
        ground_truth.yaw = ground_truth.yaw * 180.0 / M_PI;
        std::cout << "IMU data: roll = " << ground_truth.roll 
                    << ", pitch = " << ground_truth.pitch 
                    << ", yaw = " << ground_truth.yaw << std::endl;

        // 根据IMU复位标志位，决定是否需要复位IMU数据
        if(imu_reset_flag == true){
            imu_reset_flag = false;
            pose_cur = ground_truth;  // 将当前IMU数据作为复位后的初始数据
            pose_pre = ground_truth;  // 上一帧的IMU数据也设置为当前数据
        }else{
            pose_pre = pose_cur;
            IMU_POSE pose_dir;
            pose_dir.roll = ground_truth.roll - pose_pre.roll;   // 当前帧与上一帧的差值
            pose_dir.pitch = ground_truth.pitch - pose_pre.pitch; // 当前帧与上一帧的差值
            pose_dir.yaw = ground_truth.yaw - pose_pre.yaw;       // 当前帧与上一帧的差值
        }
    }
}

void IMU_HANDLER::reset_pose(){
    imu_reset_flag = true;  // 设置复位标志位为true
    // pose_cur = IMU_POSE();  // 清空当前IMU数据
    // pose_pre = IMU_POSE();  // 清空上一帧IMU数据
    // ground_truth = IMU_POSE();  // 清空地面真实IMU数据
}


