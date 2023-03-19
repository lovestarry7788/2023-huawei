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
    if (fabs(dif_rot) > 0.5 && cir < 2 && dist < 3)
        cir = Geometry::MinRadius2(dx-x0_, dy-y0_, fabs(aim_rot));
    forward = GetMaxSpeedOnCir(cir);
    forward = std::min(forward, max_forward_velocity_);

    double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
    // if (Input::frameID > 230 && Input::frameID < 260 && id_ == 3)
    //     Log::print("ToPoint", angular_velocity_, limit_r, dif_rot, dx, dy, x0_, y0_);
    // double linearV = GetLinearVelocity();
    // 速度不匹配？相差角度太大？cir变化太大？没有好的表示形式，更没有好解决方法。不碰撞自己转，乱调参数则出问题。
    if (fabs(dif_rot) < limit_r) rotate = 0; // 开始角速度减速
    else {
        rotate = max_rotate_velocity_;
        rotate = dif_rot > 0 ? -rotate : rotate;
    }
    // cir < 1 仅用于小圈转入情况，dif_rot与PI/2相近，只用于目标点在圆心处。绕圈特征：dif_rot随时间变化，以PI/2为中心，0.5幅度变化。
    // if (cir < 2.0 && fabs(fabs(dif_rot) - PI/2) < 0.3 && fabs(fabs(dif_rot) - PI/2) > 0.05) { // 调参
    //     Log::print("ToPoint2", id_, linearV, forward, fabs(fabs(dif_rot) - PI/2), cir);
    //     rotate *= 0.40*cir; // 调参 1 -> 0.4; 2->0.8 cir * 0.4
    //     // forward *= 0.5;
    // }
}

void Robot::AvoidToWall(double &forward, double &rotate) {
    double limit = CalcMaxSlowdownDist();
    double walld = DistToWall({x0_, y0_}, orient_);
    if (limit >= walld - 1.2) {
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
    // return Robot::radius_with_thing_;
    return this->carry_id_ != 0 ? Robot::radius_with_thing_ : Robot::radius_;
}

double Robot::GetMaxMass() {
    return Robot::density_ * radius_with_thing_ * radius_with_thing_;
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
    return sqrt(0.7 * max_force_ * r / GetMass()); // 0.95 转向时加速度用不完
}

double Robot::CalcTime(const Point& p) {
    double aim_r = atan2(p.y - y0_, p.x - x0_);
    double dif_r = fabs(AngleReg(aim_r - orient_));
    double dist = Dist(p.x, p.y, x0_, y0_);
    double ans = UniformVariableDist2(max_rot_force_ / GetRotInerta(), dif_r, angular_velocity_, max_rotate_velocity_);
    double linearV = GetLinearVelocity();
    double cir = std::min(1.5 * linearV / max_forward_velocity_, MinRadius(dist, dif_r)); 
    // Log::print(dist);
    dist -= 2 * cir * sin(dif_r / 2);
    // Log::print(dist);
    double t1 = ans;
    // Log::print(dif_r, cir, angular_velocity_);

    double a = max_force_ / GetMass();
    double ans_up_speed = (max_forward_velocity_ - linearV) / a;
    // Log::print(ans_up_speed, ans);
    ans = 0.97 * std::max(ans, ans_up_speed) + 0.15 * std::min(ans, ans_up_speed);
    dist -= UniformVariableDist(a, GetMaxSpeedOnCir(cir), max_forward_velocity_);
    // Log::print(dist);
    double t2 = ans;
    // Log::print(dist);

    ans += dist / max_forward_velocity_;
    // Log::print(cir, dif_r, linearV, orient_, t1, t2, ans);
    return ans;
    // x = 1/2*a*t^2
    // ans += dif_r / max_rotate_velocity_ / 0.85; // 粗略估计平均旋转速度为1/2
    // ans += dist / max_forward_velocity_ / 0.7; // 粗略估计6*0.8m/s
    // cntr = aim_r;
    // cntp = p;

}

double Robot::CalcTime(const Point& p1, const Point& p2) {
    double ans = CalcTime(p1);
    Robot a = *this;
    a.x0_ = p1.x;
    a.y0_ = p1.y;
    a.orient_ = atan2(p2.y - p1.y, p2.x - p1.x);
    a.angular_velocity_ = 0;
    a.linear_velocity_x_ = max_forward_velocity_ * cos(a.orient_);
    a.linear_velocity_y_ = max_forward_velocity_ * sin(a.orient_);
    a.carry_id_ = 1;
    ans += a.CalcTime(p2);
    return ans;
}


double Robot::GetLinearVelocity() {
    return Geometry::Length(Geometry::Vector{linear_velocity_x_, linear_velocity_y_});
}
double Robot::CalcMaxSlowdownDist() {
    return Geometry::UniformVariableDist(max_force_ / GetMaxMass(), GetLinearVelocity(), 0);
}

//void Robot::ToPoint_2(double x0, double y0, double& forward, double& rotate) {
//    double angle = atan2((y0 - y0_), (x0 - x0_)); // 计算到目标点的弧度
//    /*
//     * orient_, angle [-PI, PI]
//     * 20ms, PI / s
//    */
//    double delta_angle = (angle - orient_);
//    if (fabs(delta_angle) > acos(-1)) {
//        delta_angle = (2 * acos(-1) - fabs(delta_angle)) * (-1);
//    }
//
//    if (fabs(delta_angle) >= 0.02 * max_rotate_velocity_) {
//        rotate = max_rotate_velocity_;
//    } else {
//        rotate = delta_angle / 0.02;
//    }
//}