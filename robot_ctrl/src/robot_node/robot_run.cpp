#include "robot_node.hpp"

int main(int argc , char **argv){
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    MAIN_ROBOT robot = MAIN_ROBOT(&nh);
    ros::Rate loop_rate(200); // 200Hz
    while (ros::ok())
    {
        ros::spinOnce();
        robot.pubCmd();
        loop_rate.sleep();
    }

    return 0;
}