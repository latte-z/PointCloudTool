//
// Created by Ryan Shen on 2018-12-27.
//

#include "classifier.h"

pcl::RegionGrowing<PointT, pcl::Normal> getRegionGrowing(PointCloudT::Ptr cloud) {
    // 和triangulation中的同理，需要用到法向量，先用Normalesimation去计算
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(
            new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setKSearch(50); // get the number of k nearest neighbors used for the feature estimation
    normalEstimation.compute(*normals);

    pcl::IndicesPtr indicesPtr(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZ> passThrough;
    passThrough.setInputCloud(cloud);
    passThrough.setFilterFieldName("z");
    passThrough.setFilterLimits(0.0, 1.0);
    passThrough.filter(*indicesPtr);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> regionGrowing;
    regionGrowing.setMinClusterSize(50);
    regionGrowing.setMaxClusterSize(1000000);
    regionGrowing.setSearchMethod(tree);
    regionGrowing.setNumberOfNeighbours(30);
    regionGrowing.setInputCloud(cloud);
//    regionGrowing.setIndices(indicesPtr);
    regionGrowing.setInputNormals(normals);
    regionGrowing.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    regionGrowing.setCurvatureThreshold(1.0);

    return regionGrowing;
}
