//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_WORKBENCH_H
#define HW2023_WORKBENCH_H

#include "robot.h"
#include <iostream>

class Workbench {

public:
    size_t type_id_, frame_remain_, materials_status_, product_status_; // 工作台类型，剩余帧数，材料状态，产品状态。
    double x0_, y0_; // 坐标

    Workbench(size_t type_id, double x0, double y0, size_t frame_remain, size_t materials_status,
              size_t product_status);

    inline void init() {
        scanf("%d%lf%lf%zd%zd%zd",&type_id_, &x0_, &y0_, &frame_remain_, &materials_status_, &product_status_);
        //workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
    }

    friend class Robot;

};

#endif //HW2023_WORKBENCH_H
