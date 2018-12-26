//
// Created by Ryan Shen on 2018/11/16.
//
#pragma once

#ifndef POINT_CLOUD_TOOL_MYPOLYGONMESH_H
#define POINT_CLOUD_TOOL_MYPOLYGONMESH_H


#include <pcl/PolygonMesh.h>

class MyPolygonMesh {
public:
    MyPolygonMesh();

    ~MyPolygonMesh();

    pcl::PolygonMesh mesh;
    std::string fileName;
    std::string subName;
    bool visible = true;
};


#endif //POINT_CLOUD_TOOL_MYPOLYGONMESH_H
