//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"
#include "geometry.h"
#include "log.h"

namespace Log {
    std::ofstream ofs("debug.log");
    template<class T, class... A> void print(T&& x, A&&... a){ 
        ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
    }
}

// using namespace Geometry;

Robot::Robot(int id, int workbench, int carry_id, double time_coefficient, double collide_coefficient,
             double angular_velocity, double linear_velocity_x, double linear_velocity_y, double orient, double x0, double y0) :
             id_(id), workbench_(workbench), carry_id_(carry_id), time_coefficient_(time_coefficient), collide_coefficient_(collide_coefficient),
             angular_velocity_(angular_velocity), linear_velocity_x_(linear_velocity_x), linear_velocity_y_(linear_velocity_y), orient_(orient), x0_(x0), y0_(y0){}

// 方案1：从当前点到某点，到达时速度为0（防止走过头，更容易控制）。
void Robot::ToPoint_1(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = orient_ - aim_rot;
    // TODO: 写个类处理角度的模
    if (dif_rot > Geometry::pi) dif_rot -= 2 * Geometry::pi;
    else if (dif_rot < -Geometry::pi) dif_rot += 2 * Geometry::pi;

    static int frame = 0;
    Log::print("frame", ++frame);
    if (fabs(dif_rot) > max_orient_diff_) {
        if (fabs(dif_rot) > 1) forward = 0; // 角度太大就停下再转，防止绕圈圈
        // Log::print("change rot");
        double limit = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
        // Log::print("rot limit", limit);
        if (fabs(dif_rot) < limit) rotate = 0; // 开始减速
        else rotate = dif_rot > 0 ? -max_rotate_velocity_ : max_rotate_velocity_;
    } else {
        rotate = 0;
        // Log::print("change forward");
        double vel = Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
        double limit = Geometry::UniformVariableDist(max_force_ / GetMass(), vel, 0);
        double d = Geometry::Dist(x0_, y0_, dx, dy);
        if (limit > d) forward = 0;
        else forward = Robot::max_forward_velocity_;
    }
    Log::print(dx, dy, x0_, y0_, forward);
    Log::print(orient_, aim_rot, dif_rot, rotate);
}

// 方案2：控制通过转向圆周控制速度，除了圆周不够，都不减速
void Robot::ToPoint(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = orient_ - aim_rot;
    // TODO: 写个类处理角度的模
    if (dif_rot > Geometry::pi) dif_rot -= 2 * Geometry::pi;
    else if (dif_rot < -Geometry::pi) dif_rot += 2 * Geometry::pi;

    double dist = Geometry::Dist(x0_, y0_, dx, dy);

    
    double cir = Geometry::MinRadius(dist, dif_rot);
    forward = GetMaxSpeedOnCir(cir);
    
    double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
    if (fabs(dif_rot) < limit_r) rotate = 0; // 开始角速度减速
    else rotate = dif_rot > 0 ? -max_rotate_velocity_ : max_rotate_velocity_;

    // 不减速假设
    // double velocity = Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
    // double limit_v = Geometry::UniformVariableDist(max_force_ / GetMass(), velocity, 0);
    // if (limit_v > dist) forward = 0; // 开始线速度减速
    
    forward = std::min(forward, max_forward_velocity_);

    static int frame = 0;
    Log::print("frame", ++frame);
    Log::print(dx, dy, x0_, y0_, forward);
    Log::print(orient_, aim_rot, dif_rot, rotate);
}

double Robot::GetRadius() {
    return this->carry_id_ != 0 ? Robot::radius_with_thing_ : Robot::radius_;
}

double Robot::GetMass() {
    double r = GetRadius();
    return Robot::density_ * r * r;
}

double Robot::GetRotInerta() {
    // L = R^2 * M / 2
    double r = GetRadius();
    return GetMass() * r * r / 2;
}

double Robot::GetMaxSpeedOnCir(double r) {
    return sqrt(max_force_ * r / GetMass());
}

// zhijie
// void Robot::ToPoint(double x0, double y0, double& forward, double& rotate) {
//     double angle = atan2((y0 - y0_) , (x0 - x0_)); // 计算到目标点的弧度
//     /*
//      * orient_, angle [-pi, pi]
//      * 20ms, pi / s
//     */
//     double delta_angle = (angle - orient_);
//     if(fabs(delta_angle) > acos(-1)) {
//         delta_angle = (2 * acos(-1) - fabs(delta_angle)) * (-1);
//     }

//     if(fabs(delta_angle) >= 0.02 * max_rotate_velocity_) {
//         rotate = max_rotate_velocity_;
//     } else {
//         rotate = delta_angle / 0.02;
//     }
