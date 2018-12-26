//
// Created by Ryan Shen on 2018/11/12.
//

#ifndef POINT_CLOUD_TOOL_TRIANGULATION_H
#define POINT_CLOUD_TOOL_TRIANGULATION_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

// 存储法线计算获取的PointCloud和Kd搜索树
struct TreeCloud {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2;
};

pcl::PolygonMesh GreedyProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

pcl::PolygonMesh Poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

struct TreeCloud NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


#endif //POINT_CLOUD_TOOL_TRIANGULATION_H
