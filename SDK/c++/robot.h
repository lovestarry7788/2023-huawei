//
// Created by 刘智杰 on 2023/3/10.
//

#ifndef HW2023_ROBOT_H
#define HW2023_ROBOT_H

#include "workbench.h"
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

struct Robot {
public:
    static constexpr double seed_random_rate = 1 + 1.1e-4;
    static constexpr double radius_ = 0.45;
    static constexpr double radius_with_thing_ = 0.53;
    static constexpr double max_forward_velocity_ = 6 * seed_random_rate;
    static constexpr double max_backward_velocity_ = -2 * seed_random_rate;
    static constexpr double density_ = 50;
    static constexpr double max_force_ = 250;
    static constexpr double max_rot_force_ = 50;
    static constexpr double max_orient_diff_ = 3e-2; // 最大角度偏差
    static constexpr double max_rotate_velocity_ = Geometry::PI * seed_random_rate;

    int id_, workbench_, carry_id_; // 机器人的 id, 所属工作台id(-1不属工作台）,所携带的物品 id(0没有带物品)
    int workbench_buy_, workbench_sell_; // 买的工作台，卖的工作台。
    double time_coefficient_, collide_coefficient_; // 机器人的坐标, 时间系数, 碰撞系数
    double angular_velocity_, orient_;// 角速度, 朝向
    Geometry::Point pos_; // x坐标, y坐标
    Geometry::Vector linear_velocity_;// 线速度

    Robot(int id, int workbench_, int carry_id, double time_coefficient, double collide_coefficient,
          double angular_velocity, double linear_velocity_x, double linear_velocity_y, double orient, double x0, double y0);

    // 通过调整当前帧的姿态 (forward, rotate) 使机器人去到 (dx, dy)
    void ToPoint(Geometry::Point p, double& forward, double& rotate);

    void ToPoint_1(Geometry::Point p, double& forward, double& rotate);

    // void ToPoint_2(double dx, double dy, double& forward, double& rotate);

    void ToPoint_3(Geometry::Point p, double& forward, double& rotate);

    void ToPointTwoPoint(Geometry::Point a, Geometry::Point b, double& forward, double& rotate, int frame_a = 0);

    void AvoidToWall(double& forward, double& rotate);

    double DistToWall(Geometry::Point p, double orient);

    // 获取当前半径
    double GetRadius();

    double GetMaxMass();

    // 获取质量，不拿16，15加速度
    double GetMass();

    // 获取转动惯量，不拿2，25转动速度
    double GetRotInerta();

    double GetMaxSpeedOnCir(double r);

    // 预估走完这些点的帧
    double CalcTime(const Geometry::Point& p1, const Geometry::Point& p2);

    double CalcTime(const Geometry::Point& p);

    double CalcMaxSlowdownDist();

    double GetLinearVelocity();

    std::vector<Geometry::Point> ForecastToPoint(Geometry::Point p, int forecast_num);

    std::vector<Geometry::Point> ForecastToPoint2(Geometry::Point a, Geometry::Point b, int frame_a, int forecast_num);

    std::vector<Geometry::Point> ForecastFixed(double forward, double rotate, int forecast_num);

    friend class Workbench;
};

#endif // HW2023_ROBOT_H
