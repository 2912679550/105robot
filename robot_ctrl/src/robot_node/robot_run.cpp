#include "robot_node.hpp"

int main(int argc , char **argv){
    ros::init(argc, argv, "robot_node");
    ros::NodeHandle nh;
    MAIN_ROBOT robot();
    return 0;
}