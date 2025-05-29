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

    while (ros::ok() && !exitloop)
    {
        char temp = get_char();
        control = temp != 0 ? temp : control;
        ros::spinOnce();
        robot.robot_ctrl();

        switch(control){
            case 'q':
                std::cout << "Exit robot control loop." << std::endl;
                exitloop = true;
                break;
            case 'f':
                robot.front_side_->fix_quat();
                std::cout << "Front side IMU quaternion fixed." << std::endl;
                control = 't';
                break;
            case 't':
                IMU_POSE aixs_err;
                robot.front_side_->imu_handler_->get_aixs_err(&aixs_err, true);
                robot.front_side_->pid_handler_->Tick(aixs_err.pitch, true);
                break;
            default:
                break;
        }

        loop_rate.sleep();
    }  
    return 0;
}