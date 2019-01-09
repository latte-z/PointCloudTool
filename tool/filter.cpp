//
// Created by Ryan Shen on 2018/11/22.
//

#include <pcl/filters/passthrough.h>
#include "filter.h"

/**
 * @brief 只允许 lower_limit 到 upper_limit 范围内的点通过，过滤掉这个范围之外的点
 * @param cloud
 * @param axis: only accept "x,y,z"
 * @param lower_limit
 * @param upper_limit
 * @return
 */
PointCloudT::Ptr PassThroughFilter(PointCloudT::Ptr cloud, std::string axis, float lower_limit, float upper_limit) {
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::PassThrough<PointT> passThrough;
    passThrough.setInputCloud(cloud);
    passThrough.setFilterFieldName(axis);
    passThrough.setFilterLimits(lower_limit, upper_limit);
    passThrough.filter(*cloud_filtered);
    return cloud_filtered;
}

/**
 * @brief 统计滤波器删除离群点，主要是用于去除离散的噪声点
 * @param cloud
 * @param isNegative
 * @return
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool isNegative, int meanK, double stddev_mult) {
    PointCloudT::Ptr cloud_filtered(new PointCloudT);

    // 创建一个滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(cloud);
    statisticalOutlierRemoval.setMeanK(meanK);
    statisticalOutlierRemoval.setStddevMulThresh(stddev_mult);
    statisticalOutlierRemoval.filter(*cloud_filtered);
    if (!isNegative) {
        // 返回非离群点集
        return cloud_filtered;
    } else {
        // 同样参数再次调用，通过setNegative参数，反向选取获取离群的点集
        statisticalOutlierRemoval.setNegative(true);
        statisticalOutlierRemoval.filter(*cloud_filtered);
        return cloud_filtered;
    }
}
