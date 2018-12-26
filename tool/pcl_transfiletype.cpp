//
// Created by Ryan Shen on 2018/11/6.
//

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h> // loadPolygonFileOBJ

using namespace pcl;

int main(int argc, char **argv)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileOBJ("data/jk1.obj", mesh);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    pcl::io::savePCDFileASCII("data/jk1.pcd", *cloud);
    return 0;
}