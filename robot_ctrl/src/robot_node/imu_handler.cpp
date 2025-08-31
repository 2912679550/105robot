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
        quat.normalize();  // 归一化四元数
        quat.setX(msg->orientation.x);
        quat.setY(msg->orientation.y);
        quat.setZ(msg->orientation.z);
        quat.setW(msg->orientation.w);
        // * 先操作四元数
        quat_pre = quat_cur;  // 保存上一帧的四元数
        quat_cur = quat;  // 更新当前IMU数据的四元数
    }
}

void IMU_HANDLER::fix_quat(){
    quat_fixed = quat_cur;  // 将当前IMU数据的四元数作为目标四元数
}

float IMU_HANDLER::ger_error_yaw(tf::Quaternion *target, bool printFlag){
    IMU_POSE imu_error;
    get_aixs_err(target, &imu_error, printFlag);
    return -imu_error.pitch;
}

void IMU_HANDLER::get_aixs_err(tf::Quaternion* target ,  IMU_POSE* result , bool printFlag){
    tf::Matrix3x3 R_w_i1 = tf::Matrix3x3(*target);  // 目标四元数对应的旋转矩阵
    tf::Matrix3x3 R_w_i2 = tf::Matrix3x3(quat_cur);  // 当前四元数对应的旋转矩阵
    tf::Matrix3x3 R_i1_i2 = R_w_i1.inverse() * R_w_i2;  // 当前四元数在目标四元数对应的坐标系下的旋转矩阵

    tf::Matrix3x3 R_r1_r2 = (*imu_robot_matrix).inverse() * R_i1_i2 * (*imu_robot_matrix); // 将弧度转换为角度
    R_r1_r2.getRPY(result->roll, result->pitch, result->yaw);  // 获取欧拉角
    result->roll  = result->roll * 180.0 / M_PI;
    result->pitch = result->pitch * 180.0 / M_PI;
    result->yaw   = result->yaw * 180.0 / M_PI;
}

void IMU_HANDLER::get_aixs_err(IMU_POSE *result , bool printFlag){
    if (result == nullptr)
        return ;
        // 获取旋转矩阵
    tf::Matrix3x3 R_w_i1 = tf::Matrix3x3(quat_fixed);  // 目标四元数对应的旋转矩阵
    tf::Matrix3x3 R_w_i2 = tf::Matrix3x3(quat_cur);  // 当前四元数对应的旋转矩阵
    // 计算当前四元数在目标四元数对应的坐标系下的旋转矩阵
    tf::Matrix3x3 R_i1_i2 = R_w_i1.inverse() * R_w_i2;  // 当前四元数在目标四元数对应的坐标系下的旋转矩阵
    // 提取当前四元数在目标四元数对应的坐标系下的欧拉角
    // R_i1_i2.getRPY(result->roll, result->pitch, result->yaw);  // 获取欧拉角
    
    // 转换为机器人坐标系的描述
    tf::Matrix3x3 R_r1_r2 = (*imu_robot_matrix).inverse() * R_i1_i2 * (*imu_robot_matrix); // 将弧度转换为角度
    R_r1_r2.getRPY(result->roll, result->pitch, result->yaw);  // 获取欧拉角

    result->roll  = result->roll * 180.0 / M_PI;
    result->pitch = result->pitch * 180.0 / M_PI;
    result->yaw   = result->yaw * 180.0 / M_PI;

    if(printFlag){
        std::cout   << YELLOW_STRING
                    << "current IMU quaternion: " << quat_cur.x() << ", "
                    << quat_cur.y() << ", "
                    << quat_cur.z() << ", "
                    << quat_cur.w() << RESET_STRING
                    << std::endl;
        std::cout   << RED_STRING 
                    << "IMU axis error: roll = " << result->roll 
                    << ", pitch = " << result->pitch 
                    << ", yaw = " << result->yaw 
                    << RESET_STRING
                    << std::endl;
    }

}
