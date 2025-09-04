#include <ros/ros.h>
#include "robot_ctrl/tcp_motion_cmd.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>
#include <fstream>
#include "Timer.hpp"

using namespace std;
namespace fs = std::filesystem;

#define START_RECORD "record_start"
#define PAUSE_RECORD "pause_record"
#define END_RECORD  "record_stop"

class Record{
public:
    Record(ros::NodeHandle* nh , string record_topic_name , string ctrl_topic_name);
    bool load_topic(int file_oder);
    void save_topic();
    string filefolder_path_;
    void play_topic();

private:
    ros::NodeHandle *nh_;
    ros::Publisher  topic_pub_;   // 用于接收需要发布的话题名称
    ros::Subscriber topic_sub_;  // 用于接收需要录制的话题名称
    ros::Subscriber ctrl_sub_;   // 用于接收记录启停的接收器

    // 内置定时器
    MYTIMER timer_;
    vector<pair<float , robot_ctrl::tcp_motion_cmd>> topic_buff_;
    bool is_record_start = false;
    bool is_playing_start = false;

    void record_callback(const robot_ctrl::tcp_motion_cmd::ConstPtr& msg);
    void ctrl_callback(const std_msgs::String::ConstPtr& msg);
};
