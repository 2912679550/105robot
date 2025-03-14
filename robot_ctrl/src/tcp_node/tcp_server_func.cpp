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

