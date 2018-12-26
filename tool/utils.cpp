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
