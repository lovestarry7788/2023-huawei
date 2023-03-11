//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_ROBOT_H
#define HW2023_ROBOT_H

#pragma once
#include "workbench.h"
#include <iostream>
#include <cmath>
#include <algorithm>

struct Robot {

public:

    static constexpr double radius_ = 0.45;
    static constexpr double radius_with_thing_ = 0.53;
    static constexpr double max_forward_velocity_ = 6;
    static constexpr double max_backward_velocity_ = -2;
    const double max_rotate_velocity_ = acos(-1);

    int id_, workbench_, carry_id_; // 机器人的 id, 所携带的物品 id
    double time_coefficient_, collide_coefficient_; // 机器人的坐标, 时间系数, 碰撞系数
    double angular_velocity_, linear_velocity_, orient_, x0_, y0_; // 角速度, 线速度, 朝向, x坐标, y坐标

    Robot(int id, int workbench_, int carry_id, double time_coefficient, double collide_coefficient,
          double angular_velocity, double linear_velocity, double orient, double x0, double y0);

    // 通过调整当前帧的姿态 (forward, rotate) 使机器人去到 (dx, dy)
    void ToPoint(double dx, double dy, double &forward, double &rotate);

    friend class Workbench;
};

#endif // HW2023_ROBOT_H
