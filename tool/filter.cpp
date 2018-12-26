//
// Created by Ryan Shen on 2018/11/22.
//

#include "filter.h"

/**
 * 使用StatisticalOutlierRemoval滤波器移除离群点
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool isNegative) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建一个滤波器
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutlierRemoval;
    statisticalOutlierRemoval.setInputCloud(cloud);
    statisticalOutlierRemoval.setMeanK(50);
    statisticalOutlierRemoval.setStddevMulThresh(1.0);
    statisticalOutlierRemoval.filter(*cloud_filtered);
    if (!isNegative) {
        return cloud_filtered;
    } else {
        // 同样参数再次调用，通过setNegative参数获取离群的点集
        statisticalOutlierRemoval.setNegative(true);
        statisticalOutlierRemoval.filter(*cloud_filtered);
        return cloud_filtered;
    }
}
