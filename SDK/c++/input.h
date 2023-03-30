#ifndef HW2023_INPUT_H
#define HW2023_INPUT_H

#include "geometry.h"
#include "workbench.h"
#include "robot.h"
#include <vector>
#include <memory>
#include <iostream>

namespace Input {
    extern int frameID, coins, K;
    extern char map_[100][100];
    extern bool is_obstacle_[100][100];
    extern std::vector<std::shared_ptr<Workbench> > workbench;
    extern std::vector<std::shared_ptr<Robot> > robot;        
    extern constexpr int robot_num_ = 4;
    extern int map_number_; // 是哪张地图，-1 = unknown

    bool readUntilOK();
    void ScanMap();
    bool ScanFrame();
    void Identify_Map_Number();
};

#endif