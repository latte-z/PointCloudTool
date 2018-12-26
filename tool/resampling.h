//
// Created by Ryan Shen on 2018/11/22.
//


#ifndef POINT_CLOUD_TOOL_MLSRESAMPLING_H
#define POINT_CLOUD_TOOL_MLSRESAMPLING_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>  // kdtree搜索树
#include <pcl/surface/mls.h> // 滑动最小二乘

pcl::PointCloud<pcl::PointNormal> MLSResampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


#endif //POINT_CLOUD_TOOL_MLSRESAMPLING_H
