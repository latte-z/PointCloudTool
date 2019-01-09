//
// Created by Ryan Shen on 2018-12-27.
//

#include "classifier.h"

/**
 * @brief Basic Region Growing
 * @param cloud             原始输入点云
 * @param k                 k近邻参数
 * @param min_cluster_size  一个region最少点数量
 * @param max_cluster_size  一个region最大点数量，通常我们希望无穷大，选一个足够大的值就够了
 * @param neighbour_number  多少个点来决定一个平面
 * @param smoothness_theta  夹角阈值
 * @param curvature         曲率阈值
 * @return
 */
pcl::RegionGrowing<PointT, pcl::Normal>
getRegionGrowing(PointCloudT::Ptr cloud, int k, int min_cluster_size, int max_cluster_size,
                 unsigned int neighbour_number, float smoothness_theta, float curvature) {

    // 和triangulation中的同理，需要用到法向量，先用Normalesimation去计算
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(
            new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setKSearch(k); // get the number of k nearest neighbors used for the feature estimation
    normalEstimation.compute(*normals);

    // 增长对象
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> regionGrowing;
    // 最少点
    regionGrowing.setMinClusterSize(min_cluster_size); // example default: 50
    // 最大点，一般希望无穷大
    regionGrowing.setMaxClusterSize(max_cluster_size);
    // 用设置好的kd树
    regionGrowing.setSearchMethod(tree);
    // 参考的领域点数，即多少个点决定一个平面，决定了容错率
    // 如果设置很小，检测到的平面也很小，很大的话，可能有的点很歪
    regionGrowing.setNumberOfNeighbours(neighbour_number); // example default: 30
    // 输入检测的点云
    regionGrowing.setInputCloud(cloud);
    // regionGrowing.setIndices(indicesPtr);
    // 输入点法线
    regionGrowing.setInputNormals(normals);
    // 设置弯曲阈值，决定了是否要继续探索
    // 假设每个点都是平稳弯曲的，夹角也很小，随着探索区域增大，变化一定会超过这两个阈值
    regionGrowing.setSmoothnessThreshold(smoothness_theta);
    regionGrowing.setCurvatureThreshold(curvature);

    return regionGrowing;
}

/**
 * @brief Color based Region Growing
 * @param cloud                 输入RGB点云
 * @param min_cluster_size      region的最少点数
 * @param neighbors_distance    近邻检测阈值
 * @param point_color_diff      两点RGB差别阈值检测
 * @param region_color_diff     两域RGB差别阈值检测
 * @return
 */
pcl::RegionGrowingRGB<pcl::PointXYZRGB>
getRegionGrowingRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int min_cluster_size, float neighbors_distance,
                    float point_color_diff, float region_color_diff) {
    // color-based region growing segmentation
    // kd-tree object for searches.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud);

    // Color-based region growing clustering object.
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
    clustering.setInputCloud(cloud);
    clustering.setSearchMethod(kdtree);
    // Here, the minimum cluster size affects also the postprocessing step:
    // clusters smaller than this will be merged with their neighbors.
    clustering.setMinClusterSize(min_cluster_size);
    // Set the distance threshold, to know which points will be considered neighbors.
    clustering.setDistanceThreshold(neighbors_distance);
    // Color threshold for comparing the RGB color of two points.
    clustering.setPointColorThreshold(point_color_diff);
    // Region color threshold for the postprocessing step: clusters with colors
    // within the threshold will be merged in one.
    clustering.setRegionColorThreshold(region_color_diff);
    return clustering;
}
