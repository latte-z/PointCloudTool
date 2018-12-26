//
// Created by Ryan Shen on 2018/11/22.
//

#include "resampling.h"

pcl::PointCloud<pcl::PointNormal> MLSResampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // create kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // output has the pointnormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // 定义最小二乘法对象 mls, 第二个是存储 normals
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    // 设置最小二乘计算中需要法线的估计
    mls.setComputeNormals(true);

    // set parameters
    mls.setInputCloud(cloud);

    // MLS拟合曲线的阶数，这个阶数在构造函数里默认是2，
    // 但是参考文献给出最好选择3或者4，当然不难得出随着阶数的增加程序运行的时间也增加。
    // mls.setPolynomialOrder(3);
    mls.setPolynomialOrder(true);
    mls.setSearchMethod(tree); // 使用kdtree加速搜索
    mls.setSearchRadius(0.03); // 确定搜索的半径，半径越小拟合后曲面失真度越小

    //mls.setUpsamplingMethod(mls.NONE); // 上采样 增加较小区域的密度，对填补洞无能为力

    //mls.setUpsamplingMethod(SAMPLE_LOCAL_PLANE);
    // 需要设置半径　和　步数　mls.setUpsamplingRadius()
    // 此函数规定了点云增长的区域。可以这样理解：把整个点云按照此半径划分成若干个子点云，然后一一索引进行点云增长。
    // mls.setUpsamlingStepSize(double size) 对于每个子点云处理时迭代的步长

    //mls.setUpsamplingMethod(RANDOM_UNIFORM_DENSITY);// 它使得稀疏区域的密度增加，从而使得整个点云的密度均匀
    // 需要设置密度　 mls.setPointDensity(int desired_num);//意为半径内点的个数。

    //mls.setUpsamplingMethod(VOXEL_GRID_DILATION);// 体素格　上采样
    // 填充空洞和平均化点云的密度。它需要调用的函数为：
    // mls.setDilationVoxelSize(float voxel_size) 设定voxel的大小。

    // 重采样
    mls.process(mls_points);

    // return output
    return mls_points;
}
