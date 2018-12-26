//
// Created by Ryan Shen on 2018/11/26.
//
#pragma once
#ifndef POINT_CLOUD_TOOL_MYEXCEPTION_H
#define POINT_CLOUD_TOOL_MYEXCEPTION_H

#include <pcl/exceptions.h>

using namespace pcl;

class PCL_EXPORTS MyException : public PCLException {
public:
    MyException(const std::string &error_description,
                const char *file_name = "",
                const char *function_name = "",
                unsigned line_number = 0)
            : pcl::PCLException(error_description, file_name, function_name, line_number) {}
};


#endif //POINT_CLOUD_TOOL_MYEXCEPTION_H
