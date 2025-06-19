#include "robot_node.hpp"
#include "public.hpp"

char control = 0;

int main(int argc , char **argv){
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    MAIN_ROBOT robot = MAIN_ROBOT(&nh);
    ros::Rate loop_rate(200); // 200Hz

    control = 0;
    bool exitloop = false;
    TCP_ROBOT_CMD_TYPE cmd_msg;  // 用于人工向机器人发布控制指令
    cmd_msg.cmdType = ROBOT_STOP;  // 默认停止状态
    cmd_msg.angle_front = 0.0f;  // 前侧夹紧角度
    cmd_msg.angle_back = 0.0f;   // 后侧夹紧角度

    while (ros::ok() && !exitloop)
    {
        char temp = get_char();
        control = temp != 0 ? temp : control;
        ros::spinOnce();
        robot.robot_ctrl(false);
        // if(control != '0' && control != 0)std::cout << control << std::endl;
        switch (control)
        {
        case 'q':
            std::cout << "Exit robot control loop." << std::endl;
            exitloop = true;
            break;
        case 'f':
            robot.front_side_->fix_quat();
            std::cout << "Front side IMU quaternion fixed." << std::endl;
            control = 't';
            break;
        case 'o':
            IMU_POSE aixs_err;
            robot.front_side_->imu_handler_->get_aixs_err(&aixs_err, true);
            robot.front_side_->pid_handler_->Tick(aixs_err.pitch, true);
            break;
        // todo 以下开始为通过按键复现机器人的软件控制流程
        // * 两臂夹角控制
        // case '3':
        // case '4':
        // case '5':
        // case '6':
        // case '7':
        // case '8':
        // case '9':
        //     cmd_msg.cmdType = ROBOT_ANGLE; // 设置夹紧角度
        //     cmd_msg.angle_front = (mechAngleRange[0] + (float(int(control - '0')) - 3.0f) * (mechAngleRange[1] - mechAngleRange[0]) / (9.0f - 3.0f));
        //     cmd_msg.angle_back = (mechAngleRange[0] + (float(int(control - '0')) - 3.0f) * (mechAngleRange[1] - mechAngleRange[0]) / (9.0f - 3.0f));
        //     std::cout << "Set robot front angle to " << cmd_msg.angle_front << " and back angle to " << cmd_msg.angle_back << std::endl;
        //     robot.cmd_hand_maked(&cmd_msg);
        //     control = '0'; // 重置控制字符
        //     break;
        // * 舵轮标定
        case 'c':
            cmd_msg.cmdType = ROBOT_CALI; // 舵轮标定
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        // * wasd 控制机器人运动
        case 'w':
            cmd_msg.cmdType = ROBOT_MOTION; // 正常运动
            cmd_msg.v_axi = 0.02f;          // 设置轴向速度
            cmd_msg.v_cir = 0.0f;           // 设置周向速度
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        case 's':
            cmd_msg.cmdType = ROBOT_MOTION; // 正常运动
            cmd_msg.v_axi = -0.02f;         // 设置轴向速度
            cmd_msg.v_cir = 0.0f;           // 设置周向速度
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        case 'a':
            cmd_msg.cmdType = ROBOT_MOTION; // 正常运动
            cmd_msg.v_axi = 0.0f;           // 设置轴向速度
            cmd_msg.v_cir = -0.02f;         // 设置周向速度
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        case 'd':
            cmd_msg.cmdType = ROBOT_MOTION; // 正常运动
            cmd_msg.v_axi = 0.0f;           // 设置轴向速度
            cmd_msg.v_cir = 0.02f;          // 设置周向速度
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        // * 夹紧控制
        case 't':
            cmd_msg.cmdType = ROBOT_TIGHT_EN; // 启用夹紧
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        case 'l':
            cmd_msg.cmdType = ROBOT_TIGHT_DIS; // 松开夹紧
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        case 'r':
            cmd_msg.cmdType = ROBOT_STOP; // 停止运动
            robot.cmd_hand_maked(&cmd_msg);
            control = '0'; // 重置控制字符
            break;
        default:
            break;
        }

        loop_rate.sleep();
    }  
    return 0;
}