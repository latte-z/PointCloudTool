//
// Created by Ryan Shen on 2018/11/21.
//

#include "projection.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr GetProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud, int type) {
    /**
     * Different type correspond different 模型数据
     */

    // originCloud: cloud before projection
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 定义模型数据对象，并填充对应的数据 -> With X=Y=0,Z=1
    // we use a plane model, with ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the X-Y plane.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
//    coefficients->values[0] = 0.5; //0.5
//    coefficients->values[1] = 1;   //0
//    coefficients->values[2] = 1;   //0
//    coefficients->values[3] = 0;   //0
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> projectInliers;
    // set object projection model
    projectInliers.setModelType(pcl::SACMODEL_PLANE);
    // input point cloud
    projectInliers.setInputCloud(originCloud);
    // 设置模型对应系数
    projectInliers.setModelCoefficients(coefficients);
    // 执行投影滤波存储结果
    projectInliers.filter(*projectedCloud);

    return projectedCloud;
}
