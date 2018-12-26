//
// Created by Ryan Shen on 2018/11/22.
//

#include "io.h"

std::string getSubName(std::string file_name) {
    std::string sub_name;
    for (auto i = file_name.end() - 1; *i != '/'; i--)
        sub_name.insert(sub_name.begin(), *i);
    return std::string();
}
