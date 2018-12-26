#include <utility>

//
// Created by Ryan Shen on 2018/11/12.
//

#include "triangulation.h"

struct TreeCloud NormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    struct TreeCloud treeCloud;
    // Normal esitimation: http://pointclouds.org/documentation/tutorials/normal_estimation.php
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud); // 用cloud创建tree对象
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(8); // 临近点个数
    n.compute(*normals); // 估计法线存储到其中
    // normals 不应包含点法线和面曲率

    // 连接XYZ和法线区域
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); // 连接字段
    // cloud_with_normals = cloud + normals;

    // 创建搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // 点云构建搜索树
    treeCloud.cloud_with_normals = cloud_with_normals;
    treeCloud.tree2 = tree2;
    return treeCloud;
}

pcl::PolygonMesh GreedyProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    struct TreeCloud treeCloud = NormalEstimation(std::move(cloud));

    // 初始化对象
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化的对象
    pcl::PolygonMesh triangles; // 存储最终三角化的模型

    // 设置连接点之间的最大距离(即最大的边长)
    gp3.setSearchRadius(50); // 搜索半径，决定了重建后三角形大小（即三角形最大边长）

    // 设置参数典型值
    gp3.setMu(2.5); // 设置被样本点搜索其近邻点最远距离为2.5，为了使用点云密度变化。mu一般取值为2.5-3。
    gp3.setMaximumNearestNeighbors(100); // 临近点个数阈值设定 80-100
    gp3.setMaximumSurfaceAngle(M_PI / 2); //两点法向量角度差大于这个值就不连接三角形，保证点云的局部平滑约束，如果一个点是尖锐点就不会和它周围点构成三角形
    gp3.setMinimumAngle(M_PI / 36); // 三角形最小角度阈值
    gp3.setMaximumAngle(5 * M_PI / 6); // 三角形最大角度阈值
    gp3.setNormalConsistency(false); // 法向量是否连续变化，一般是false。即保证法线朝向一致。除非点云全局光滑，如：球

    // 获取结果
    gp3.setInputCloud(treeCloud.cloud_with_normals); // 设置输入为有向点云
    gp3.setSearchMethod(treeCloud.tree2); // 搜索方式
    gp3.reconstruct(triangles); // 重建提取三角化

    // 额外顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // save result
    // pcl::io::saveVTKFile("../vtkfile.vtk", triangles);

    return triangles;
}

pcl::PolygonMesh Poisson(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    struct TreeCloud treeCloud = NormalEstimation(std::move(cloud));

    pcl::Poisson<pcl::PointNormal> pn;
    pn.setConfidence(true); // 是否使用法向量大小作为置信信息。如果false则所有法向量归一化
    pn.setDegree(2); // 设置参数degree[1,5]，值越大越精细，耗时越久
    pn.setDepth(8); // 树的最大深度，求解2^d x 2 ^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
    pn.setIsoDivide(8); // 用于提取ISO等值面算法的深度
    pn.setManifold(true); // 是否添加多边形的重心，当多边形三角化时，设置流行标志，如果设置为true，则对多边形进行吸粉三角化时添加重心，设置false不添加
    pn.setOutputPolygons(false); // 是否输出多边形网络（而不是三角化移动立方体的结果）
    pn.setSamplesPerNode(3.0); // 设置落入一个八叉树节点中的样本点的最小数量。无噪声[1.0-5.0]，有噪声[15.-20.]平滑
    pn.setScale(1.25); // 设置用于重构的立方体直径和样本边界立方体直径的比率
    pn.setSolverDivide(8); // 设置求解线性方程组的Gauss-Seidel迭代方法的深度
    // pn.setIndices();
    pn.setSearchMethod(treeCloud.tree2);
    pn.setInputCloud(treeCloud.cloud_with_normals);
    pcl::PolygonMesh triangles;
    pn.performReconstruction(triangles);
    return triangles;
}

