//
// Created by Ryan Shen on 2018/11/22.
//
#pragma once
#ifndef POINT_CLOUD_TOOL_FILTER_H
#define POINT_CLOUD_TOOL_FILTER_H

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pojo/MyPointCloud.h"

PointCloudT::Ptr PassThroughFilter(PointCloudT::Ptr cloud, std::string axis, float lower_limit, float upper_limit);

/**
 * 使用StatisticalOutlierRemoval滤波器移除离群点
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool isNegative, int meanK, double stddev_mult);


#endif //POINT_CLOUD_TOOL_FILTER_H
