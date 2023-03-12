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

void Robot::ToPoint(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = orient_ - aim_rot;
    static int frame = 0;
    Log::print("frame", ++frame);
    if (fabs(dif_rot) > max_orient_diff_) {
        forward = 0;
        Log::print("change rot");
        double limit = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
        if (dif_rot < limit) rotate = 0;
        else rotate = dif_rot > 0 ? max_rotate_velocity_ : -max_rotate_velocity_;
    } else {
        rotate = 0;
        Log::print("change forward");
        double vel = Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
        double limit = Geometry::UniformVariableDist(max_force_ / GetMass(), vel, 0);
        double d = Geometry::Dist(x0_, y0_, dx, dy);
        if (limit > d) forward = 0;
        else forward = Robot::max_forward_velocity_;
    }
    Log::print(dx, dy, x0_, y0_, forward);
    Log::print(orient_, aim_rot, rotate);
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