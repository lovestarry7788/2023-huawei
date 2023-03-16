//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"
#include "geometry.h"
#include "log.h"

using namespace Geometry;

Robot::Robot(int id, int workbench, int carry_id, double time_coefficient, double collide_coefficient,
             double angular_velocity, double linear_velocity_x, double linear_velocity_y, double orient, double x0, double y0) :
             id_(id), workbench_(workbench), carry_id_(carry_id), time_coefficient_(time_coefficient), collide_coefficient_(collide_coefficient),
             angular_velocity_(angular_velocity), linear_velocity_x_(linear_velocity_x), linear_velocity_y_(linear_velocity_y), orient_(orient), x0_(x0), y0_(y0){}

// 方案1：从当前点到某点，到达时速度为0（防止走过头，更容易控制）。
void Robot::ToPoint_1(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = AngleReg(orient_ - aim_rot);

    // static int frame = 0;
    // Log::print("frame", ++frame);
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
    // Log::print(dx, dy, x0_, y0_, forward);
    // Log::print(orient_, aim_rot, dif_rot, rotate);
}

// 方案2：控制通过转向圆周控制速度，除了圆周不够，都不减速
void Robot::ToPoint(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = AngleReg(orient_ - aim_rot);

    double dist = Geometry::Dist(x0_, y0_, dx, dy);


    double cir = Geometry::MinRadius(dist, fabs(dif_rot)); 
    forward = GetMaxSpeedOnCir(cir);

    double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
    if (fabs(dif_rot) < limit_r) rotate = 0; // 开始角速度减速
    else rotate = dif_rot > 0 ? -max_rotate_velocity_ : max_rotate_velocity_;

    // 不减速假设
    // double velocity = Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
    // double limit_v = Geometry::UniformVariableDist(max_force_ / GetMass(), velocity, 0);
    // if (limit_v > dist) forward = 0; // 开始线速度减速

    forward = std::min(forward, max_forward_velocity_);
    // Log::print(id_, dist, dif_rot, cir, forward, rotate);
    // static int frame = 0;
    // Log::print("frame", ++frame);
    // Log::print(dx, dy, x0_, y0_, forward);
    // Log::print(orient_, aim_rot, dif_rot, rotate);
}

void Robot::AvoidToWall(double &forward, double &rotate) {
    double limit = CalcSlowdownDist();
    double walld = DistToWall({x0_, y0_}, orient_);
    if (limit >= walld - 1.1) {
        forward = 0;
    }
}

double Robot::DistToWall(Point p, double orient) {
    double mind = 100;
    Vector ori{cos(orient), sin(orient)};
    const static std::vector<std::pair<Point, double>> wall{
            {{0,0}, 0},
            {{50,0}, PI/2},
            {{50,50}, PI},
            {{0,50}, -PI/2},
    };
    for (const auto& [wp, wo] : wall) {
        Point sec = GetLineIntersection2(p, ori, wp, {cos(wo), sin(wo)});
        // if (Input::frame)
        if (Dot(sec - p, ori) > 0) {
            mind = std::min(mind, Length(sec - p));
        }
    }
    return mind;
}

double Robot::GetRadius() {
    return Robot::radius_with_thing_;
    // return this->carry_id_ != 0 ? Robot::radius_with_thing_ : Robot::radius_;
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

// Todo:
int Robot::CalcTime(const std::vector<Geometry::Point>& route) {
    // 不考虑加速过程
    Point cntp = {x0_, y0_};
    double cntr = orient_;
    double ans = 0;
    for (const auto& p : route) {
        double aim_r = atan2(p.y - cntp.y, p.x - cntp.x);
        double dif_r = AngleReg(aim_r - cntr);
        double dist = Dist(p.x, p.y, cntp.x, cntp.y);
        // x = 1/2*a*t^2
        ans += dif_r / max_rotate_velocity_ / 0.85; // 粗略估计平均旋转速度为1/2
        ans += dist / max_forward_velocity_ / 0.7; // 粗略估计6*0.8m/s
        cntr = aim_r;
        cntp = p;
    }
    return ans * 50;
}

double Robot::GetLinearVelocity() {
    return Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
}
double Robot::CalcSlowdownDist() {
    return Geometry::UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), 0);
}

// zhijie
// void Robot::ToPoint(double x0, double y0, double& forward, double& rotate) {
//     double angle = atan2((y0 - y0_) , (x0 - x0_)); // 计算到目标点的弧度
//     /*
//      * orient_, angle [-PI, PI]
//      * 20ms, PI / s
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
