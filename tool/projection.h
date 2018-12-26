//
// Created by Ryan Shen on 2018/11/21.
//

#ifndef POINT_CLOUD_TOOL_PROJECTION_H
#define POINT_CLOUD_TOOL_PROJECTION_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr GetProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud, int type);

#endif //POINT_CLOUD_TOOL_PROJECTION_H
