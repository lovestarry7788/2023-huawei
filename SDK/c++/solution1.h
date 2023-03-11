//
// Created by gllonkxc on 2023/3/11.
//

#ifndef CODECRAFTSDK_SOLUTION1_H
#define CODECRAFTSDK_SOLUTION1_H

#include "robot.h"
#include "workbench.h"
#include "map_.h"
#include <iostream>
#include <vector>
#include <memory>
using namespace std;

inline void solve(int frameID, int coins, int K, vector<shared_ptr<Workbench>> workbench, vector<shared_ptr<Robot>> robot) {
    output_frameID(frameID);
    for(int i = 0; i < robot.size(); i++) {
        forward(robot[i]->id_, 6);
    }
    outputOK();
}

#endif //CODECRAFTSDK_SOLUTION1_H
