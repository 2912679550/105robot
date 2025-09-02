#include "record.hpp"
#include "yaml-cpp/yaml.h"

Record::Record(ros::NodeHandle* nh ,string record_topic_name , string ctrl_topic_name){
    if(nh == nullptr)
        nh_ = new ros::NodeHandle;
    else
        nh_ = nh;
    topic_pub_ = nh_->advertise<robot_ctrl::tcp_motion_cmd>(record_topic_name, 1);
    topic_sub_ = nh_->subscribe(record_topic_name, 1, &Record::record_callback, this);
    ctrl_sub_ = nh_->subscribe(ctrl_topic_name, 1, &Record::ctrl_callback, this);
    timer_.reset();
}

/**
 * @brief 
 * 
 * @param msg 
 */
void Record::record_callback(const robot_ctrl::tcp_motion_cmd::ConstPtr& msg){
    if(is_record_start){
        float time = timer_.get_ms_duration();
        auto cmd = *msg;
        topic_buff_.push_back(std::make_pair(time, cmd));
    }
}

/**
 * @brief 
 * 
 * @param msg 
 */
void Record::ctrl_callback(const std_msgs::String::ConstPtr& msg){
    string ctrl = msg->data;
    if(ctrl == START_RECORD && is_record_start == false){
        if(is_playing_start) return;
        topic_buff_.clear(); // 清空容器
        is_record_start = true;
        timer_.reset();
        timer_.get_ms_duration();
    }
    else if(ctrl == END_RECORD && is_record_start == true){
        if(is_playing_start) return;
        is_record_start = false;
        timer_.reset();
        save_topic();
    }else if(ctrl.length() == 1){
        if(is_record_start == false){
            bool flag = load_topic(std::stoi(ctrl));
            if(flag == true){
                is_playing_start = true;
            }
        }
    }
}

void Record::save_topic(){
    vector<string> files;
    std::cout << "File Folder Path: " << filefolder_path_ << std::endl;
    if(!filefolder_path_.empty()){
        for (const auto &entry : fs::directory_iterator(filefolder_path_))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml")
            {
                files.push_back(entry.path().filename().string());
            }
        }
    }
    string now_name = to_string(files.size() + 1) + ".yaml";
    string full_name = filefolder_path_ + "/" + now_name;
    std::cout << "full name: " << full_name << std::endl;
    std::cout << "All Cmd Nums: " << topic_buff_.size() << std::endl;
    std::ofstream ofs(full_name);

    if(!ofs.is_open()){
        ROS_ERROR("Error record file path");
        return;
    }

    int idx = 1;
    for( const auto& item : topic_buff_){
        float time_point = item.first;
        const auto &cmd = item.second;
        ofs << "- id: " << idx << "\n";
        ofs << "  time: " << time_point << "\n";
        ofs << "  topic: " << "\n";
        ofs << "    cmdType: " << cmd.cmdType << "\n";
        ofs << "    v_axi: " << cmd.v_axi << "\n";
        ofs << "    v_cir: " << cmd.v_cir << "\n";
        ofs << "    dia_front: " << cmd.dia_front << "\n";
        ofs << "    dia_back: " << cmd.dia_back << "\n";
        ofs << "    dir_tight_front: " << cmd.dir_tight_front << "\n";
        ofs << "    dir_tight_back: " << cmd.dir_tight_back << "\n";
        ofs << "    push_length_f: " << cmd.push_length_f << "\n";
        ofs << "    push_length_b: " << cmd.push_length_b << "\n";
        idx++;
    }
    ofs.close();
}

bool Record::load_topic(int file_oder){
    vector<string> files;
    if(!filefolder_path_.empty()){
        for (const auto &entry : fs::directory_iterator(filefolder_path_))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".yaml")
            {
                files.push_back(entry.path().filename().string());
            }
        }
    }else{
        ROS_INFO("Load File Error!!!!");
        return false;
    }
    if(file_oder > files.size()){
        ROS_INFO("File num Error!!!!");
        return false;
    }

    std::sort(files.begin(), files.end());
    // 按顺序打印files中的所有文件名
    // for(const auto& name : files) {
    // std::cout << name << std::endl;
    // }
    string full_name = filefolder_path_ + "/" + files[file_oder - 1];
    std::ifstream ifs(full_name);
    if(!ifs.is_open()){
        ROS_INFO("File Open Error!!");
        return false;
    }
    std::cout << "Loading File: " << full_name << std::endl;

    // 文件以及路径ok , 开始读取
    YAML::Node root = YAML::LoadFile(full_name);
    topic_buff_.clear();
    float time = 0.0;

    for (const auto& node : root) {
        time = time + node["time"].as<float>();
        robot_ctrl::tcp_motion_cmd cmd;
        cmd.cmdType = node["topic"]["cmdType"].as<std::string>();
        cmd.v_axi = node["topic"]["v_axi"].as<float>();
        cmd.v_cir = node["topic"]["v_cir"].as<float>();
        cmd.dia_front = node["topic"]["dia_front"].as<float>();
        cmd.dia_back = node["topic"]["dia_back"].as<float>();
        cmd.dir_tight_front = node["topic"]["dir_tight_front"].as<float>();
        cmd.dir_tight_back = node["topic"]["dir_tight_back"].as<float>();
        cmd.push_length_f = node["topic"]["push_length_f"].as<float>();
        cmd.push_length_b = node["topic"]["push_length_b"].as<float>();
        topic_buff_.push_back(std::make_pair(time, cmd));
    }
    return true;
}

void Record::play_topic(){
    static int cmd_id = 0;
    if (is_playing_start == false){
        cmd_id = 0;
        return;
    }
    if(timer_.get_ms() >= topic_buff_[cmd_id].first){
        topic_pub_.publish(topic_buff_[cmd_id].second);
        cmd_id++;
    }
    if(cmd_id > topic_buff_.size() - 1)
        is_playing_start = false;
}
