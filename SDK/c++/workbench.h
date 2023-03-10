//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_WORKBENCH_H
#define HW2023_WORKBENCH_H

#include "robot.h"
#include <iostream>

class Workbench {

public:
    size_t typeid_, frame_remain_, materials_status_, product_status_; // 工作台类型，剩余帧数，材料状态，产品状态。
    double x0_, y0_; // 坐标

    Workbench(size_t type_id, double x0, double y0, size_t type_id, size_t frame_remain, size_t material_status,
              size_t product_status);

    ~Workbench();

    friend class Robot;
};

#endif //HW2023_WORKBENCH_H
