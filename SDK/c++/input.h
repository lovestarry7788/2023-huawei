#ifndef HW2023_INPUT_H
#define HW2023_INPUT_H

#include "workbench.h"
#include "robot.h"
#include <vector>
#include <memory>
#include <iostream>

namespace Input {
    extern int frameID, coins, K;
    extern char map_[101][101];
    extern std::vector<std::shared_ptr<Workbench> > workbench;
    extern std::vector<std::shared_ptr<Robot> > robot;        
    constexpr int robot_num_ = 4;

    bool readUntilOK();
    void ScanMap();

    bool ScanFrame();
};

#endif