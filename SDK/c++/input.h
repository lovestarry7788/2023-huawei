#ifndef HW2023_INPUT_H
#define HW2023_INPUT_H

#include "geometry.h"
#include "workbench.h"
#include "robot.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <cstring>
#include <cstdio>
#include <unordered_map>
#include <map>
#include <iostream>
#include <utility>
#include <functional>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <memory>
#include <climits>

namespace Input {
    static constexpr int robot_num_ = 4;
    static constexpr int map_size_ = 100;
    extern int frameID, coins, K;
    extern char map_[map_size_][map_size_];
//    extern int map_id_[map_size_][map_size_];
    extern bool is_obstacle_[map_size_][map_size_];
    extern std::vector<std::shared_ptr<Workbench> > workbench;
    extern std::vector<std::shared_ptr<Robot> > robot;
    extern int map_number_; // 是哪张地图，-1 = unknown

    bool readUntilOK();
    void ScanMap();
    bool ScanFrame();
    void Identify_Map_Number();
};

#endif