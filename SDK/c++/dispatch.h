#ifndef HW2023_DISPATCH_H
#define HW2023_DISPATCH_H

#include "input.h"
#include "geometry.h"
#include "output.h"
#include <set>

// 负责运送机器人过程，包含避障
// 先购买/出售，再addplan，再调用FrameEnd交给dispatch处理topoint
namespace Dispatch {
    using namespace Geometry;
    extern std::vector<std::set<Point>> forecast_;
    extern std::vector<Point> plan_;
    void AddPlan(int robot_id, int workbench_id);
    void FrameEnd();
}

#endif