#include "tcp_server.hpp"

void buf_split(std::vector<std::string> &return_data_vec, const std::string &str, const std::string &pattern)
{
    char * strc = new char[strlen(str.c_str())+1];
    strcpy(strc, str.c_str());   //string转换成C-string
    char* temp = strtok(strc, pattern.c_str());
    while(temp != nullptr)
    {
        return_data_vec.push_back(std::string(temp));
        temp = strtok(nullptr, pattern.c_str());
    }
    delete[] strc;
    return;
}

void motion_val_callback(const ROBOT_TCP_VAL_CPTR& msg)
{
    static int countt=0;
    countt++;
    if(isConnected == 1)
    {
        std::string speed_data_string;
        char* str;
        speed_data_string.append("traj");
        //上位机的格式：sprintf(dbgStr, "fan\t%.3f\t%.3f\t%.3f\r\n", param_sample[0], param_sample[1], param_sample[2]);
        // for (size_t i = 0; i < msg->data.size(); i++) //将轨迹数据合并成一个字符串
        // {
        //     speed_data_string.append("\t");
        //     sprintf(str,"%.3f",msg->data[i]);
        //     //fan_data_string.append(std::to_string(msg->data[i]));
        //     speed_data_string.append(str);
        // }
        speed_data_string.append("\r\n");
        char* traj_data_char = new char[strlen(speed_data_string.c_str())+1];
        strcpy(traj_data_char,speed_data_string.c_str());//将string类型转化为C语言中的char*类型
        if(countt>10)//降低发送频率
        {
            // std::cout<<fan_data_char<<std::endl; //debug
            write(confd, traj_data_char, strlen(traj_data_char));
            countt=0;
        }

        delete[] traj_data_char;
    }
    return;
}