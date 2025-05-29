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

        // * 转换为RPY开始操作
        // 转换为RPY
        tf::Matrix3x3 tempRota = tf::Matrix3x3(quat);
        tempRota.getRPY(ground_truth.roll, ground_truth.pitch, ground_truth.yaw);  // 获取欧拉角
        // 此时的单位为弧度，转化为角度
        ground_truth.roll = ground_truth.roll * 180.0 / M_PI;
        ground_truth.pitch = ground_truth.pitch * 180.0 / M_PI;
        ground_truth.yaw = ground_truth.yaw * 180.0 / M_PI;

        // if(imu_robot_matrix != nullptr){
        //     // 首先将ground_truth创建为一个tf库的列向量
        //     tf::Vector3 ground_truth_vector(ground_truth.roll, ground_truth.pitch, ground_truth.yaw);
        //     // 然后将其转换为机器人坐标系下的IMU数据
        //     tf::Vector3 imu_robot_vector = (*imu_robot_matrix) * ground_truth_vector;  // 使用IMU到机器人坐标系的旋转矩阵进行转换
        //     // 更新ground_truth为机器人坐标系下的IMU数据
        //     ground_truth.roll = imu_robot_vector.x();
        //     ground_truth.pitch = imu_robot_vector.y();
        //     ground_truth.yaw = imu_robot_vector.z();
        // }
        // std::cout << "IMU data: roll = " << ground_truth.roll 
        //             << ", pitch = " << ground_truth.pitch 
        //             << ", yaw = " << ground_truth.yaw << std::endl;

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
}

void IMU_HANDLER::fix_quat(){
    quat_fixed = quat_cur;  // 将当前IMU数据的四元数作为目标四元数
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

    result->roll = result->roll * 180.0 / M_PI;
    result->pitch = result->pitch * 180.0 / M_PI;
    result->yaw = result->yaw * 180.0 / M_PI;

    if(printFlag){
        std::cout   << RED_STRING 
                    << "IMU axis error: roll = " << result->roll 
                    << ", pitch = " << result->pitch 
                    << ", yaw = " << result->yaw 
                    << RESET_STRING
                    << std::endl;
    }

}
