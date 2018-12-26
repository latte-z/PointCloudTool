//
// Created by Ryan Shen on 2018/11/16.
//
#pragma once

#ifndef POINT_CLOUD_TOOL_MYPOINTCLOUD_H
#define POINT_CLOUD_TOOL_MYPOINTCLOUD_H

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class MyPointCloud {
public:
    MyPointCloud();

    ~MyPointCloud();

    PointCloudT::Ptr cloud;
    std::string fileName;
    std::string subName;
    bool visible = true;

};


#endif //POINT_CLOUD_TOOL_MYPOINTCLOUD_H
