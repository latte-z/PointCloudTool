//
// Created by Ryan Shen on 2018-12-27.
//
#pragma once
#ifndef POINT_CLOUD_TOOL_CLASSIFIER_H
#define POINT_CLOUD_TOOL_CLASSIFIER_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include "pojo/MyPointCloud.h"


class Classifier {

};

pcl::RegionGrowing<PointT, pcl::Normal> getRegionGrowing(PointCloudT::Ptr cloud);


#endif //POINT_CLOUD_TOOL_CLASSIFIER_H
