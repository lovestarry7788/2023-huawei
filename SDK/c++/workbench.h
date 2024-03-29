//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_WORKBENCH_H
#define HW2023_WORKBENCH_H

#include "robot.h"
#include "geometry.h"
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

struct Workbench {

public:
    int type_id_, frame_remain_, materials_status_, product_status_; // 工作台类型，剩余帧数，材料状态，产品状态。
    Geometry::Point pos_;
    // double x0_, y0_; // 坐标

    Workbench(int type_id, double x0, double y0, int frame_remain, int materials_status,
              int product_status);

    bool TryToBuy(int carry_id, double time_to_buy);
    bool TryToSell(int carry_id);
    int ItemsAreMissing();

    friend class Robot;
};

#endif // HW2023_WORKBENCH_H
