//
// Created by Ryan Shen on 2018/11/24.
//

#include "utils.h"

bool FindInVector(int target, const vector<int> &collection) {
    for (int i : collection) {
        if (target == i)
            return true;
    }
    return false;
}

//void setPointColor(PointCloudT cloud, uint8_t r, uint8_t g, uint8_t b) {
//    for (size_t i = 0; i < cloud.size(); i++) {
//        cloud.points[i].r = r;
//        cloud.points[i].g = g;
//        cloud.points[i].b = b;
//        cloud.points[i].a = 255;
//    }
//}
