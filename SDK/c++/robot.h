//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_ROBOT_H
#define HW2023_ROBOT_H

#include "workbench.h"
#include "geometry.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>

struct Robot {

public:
    static constexpr double radius_ = 0.45;
    static constexpr double radius_with_thing_ = 0.53;
    static constexpr double max_forward_velocity_ = 6;
    static constexpr double max_backward_velocity_ = -2;
    static constexpr double density_ = 50;
    static constexpr double max_force_ = 250;
    static constexpr double max_rot_force_ = 50;
    static constexpr double max_orient_diff_ = 3e-2; // 最大角度偏差
    const double max_rotate_velocity_ = Geometry::PI;

    int id_, workbench_, carry_id_; // 机器人的 id, 所携带的物品 id(0没有带物品)
    double time_coefficient_, collide_coefficient_; // 机器人的坐标, 时间系数, 碰撞系数
    double angular_velocity_, linear_velocity_x_, linear_velocity_y_, orient_, x0_, y0_; // 角速度, 线速度, 朝向, x坐标, y坐标

    Robot(int id, int workbench_, int carry_id, double time_coefficient, double collide_coefficient,
          double angular_velocity, double linear_velocity_x, double linear_velocity_y, double orient, double x0, double y0);

    // 通过调整当前帧的姿态 (forward, rotate) 使机器人去到 (dx, dy)
    void ToPoint(double dx, double dy, double& forward, double& rotate);

    void ToPoint_1(double dx, double dy, double& forward, double& rotate);

    // 获取当前半径
    double GetRadius();

    // 获取质量，不拿16，15加速度
    double GetMass();

    // 获取转动惯量，不拿2，25转动速度
    double GetRotInerta();

    double GetMaxSpeedOnCir(double r);

    // 预估走完这些点的帧
    int CalcTime(const std::vector<Geometry::Point>& route);

    friend class Workbench;
};

#endif // HW2023_ROBOT_H
