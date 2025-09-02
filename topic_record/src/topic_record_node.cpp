#include "record.hpp"
#include "channel.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "record_node");
    ros::NodeHandle nh("~");

    std::string need_record_topic;
    std::string ctrl_record_topic;
    std::string record_filefolder;
    nh.getParam("need_record_topic", need_record_topic);
    nh.getParam("ctrl_record_topic", ctrl_record_topic);
    nh.getParam("record_filefolder", record_filefolder);
    std::cout<< " ========= start record =========="<<std::endl;
    std::cout << need_record_topic << std::endl;
    std::cout << ctrl_record_topic << std::endl;
    std::cout << record_filefolder << std::endl;

    Record my_record(nullptr , need_record_topic , ctrl_record_topic);
    my_record.filefolder_path_ = record_filefolder;
    while (ros::ok())
    {
        my_record.play_topic();
        ros::spinOnce();
    }
    return 0;
}

