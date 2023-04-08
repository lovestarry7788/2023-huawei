//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"
#include "geometry.h"
#include "log.h"
#include "wayfinding.h"
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

using namespace Geometry;

Robot::Robot(int id, int workbench, int carry_id, double time_coefficient, double collide_coefficient,
             double angular_velocity, double linear_velocity_x, double linear_velocity_y, double orient, double x0, double y0) :
        id_(id), workbench_(workbench), carry_id_(carry_id), time_coefficient_(time_coefficient), collide_coefficient_(collide_coefficient),
        angular_velocity_(angular_velocity), linear_velocity_({linear_velocity_x, linear_velocity_y}), orient_(orient), pos_({x0, y0}) {}

// 方案1：从当前点到某点，到达时速度为0（防止走过头，更容易控制）。
void Robot::ToPoint_1(Point p, double& forward, double& rotate) {
    double aim_rot = Geometry::Angle(p - pos_);
    double dif_rot = AngleReg(orient_ - aim_rot);

    // static int frame = 0;
    // Log::print("frame", ++frame);
    rotate = 0;
    // Log::print("change forward");
    double vel = Geometry::Length(linear_velocity_);
    double limit = Geometry::UniformVariableDist(max_force_ / GetMass(), vel, 0);
    double d = Geometry::Length(p - pos_);
    if (limit > d) forward = 0;
    else forward = Robot::max_forward_velocity_;
    if (fabs(dif_rot) > max_orient_diff_) {
        // 2 max
        // 2.6
        if (fabs(dif_rot) > 0.1) forward = 0; // 角度太大就停下再转，防止绕圈圈

//        if (fabs(dif_rot) > eps) forward = 0; // 角度太大就停下再转，防止绕圈圈
        // else forward = max_forward_velocity_ * std::max(0.0, 3.5-1 / 0.6 * fabs(dif_rot));
        // Log::print("change rot");
        double limit = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
        // Log::print("rot limit", limit);
        if (fabs(dif_rot) < limit) rotate = 0; // 开始减速
        else rotate = dif_rot > 0 ? -max_rotate_velocity_ : max_rotate_velocity_;
    }
    // Log::print(dx, dy, x0_, y0_, forward);
    // Log::print(orient_, aim_rot, dif_rot, rotate);
}

// 方案2：控制通过转向圆周控制速度，除了圆周不够，都不减速
void Robot::ToPoint(Point p, double& forward, double& rotate) {
    double aim_rot = Geometry::Angle(p - pos_);
    double dif_rot = AngleReg(aim_rot - orient_);
    double dist = Geometry::Length(p - pos_);

    double cir = Geometry::MinRadius(dist, fabs(dif_rot));
    // if (fabs(dif_rot) > 0.5 && cir < 2 && dist < 3)
    //     cir = Geometry::MinRadius2(dx-x0_, dy-y0_, fabs(aim_rot));
    // forward = GetMaxSpeedOnCir(cir);
    // forward = std::min(forward, max_forward_velocity_);
    double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
    // if (Input::frameID > 230 && Input::frameID < 260 && id_ == 3)
    //     Log::print("ToPoint", angular_velocity_, limit_r, dif_rot, dx, dy, x0_, y0_);
    // double linearV = GetLinearVelocity();
    // 速度不匹配？相差角度太大？cir变化太大？没有好的表示形式，更没有好解决方法。不碰撞自己转，乱调参数则出问题。
    // Log::print(fabs(dif_rot), limit_r, dist);
    if (fabs(dif_rot) <= limit_r) {
        rotate = 0; // 开始角速度减速
        forward = max_forward_velocity_;
    }
        // else if (fabs(dif_rot) <= limit_r){
        //     rotate = -max_rotate_velocity_;
        //     forward = max_forward_velocity_;
        // }
    else {
        rotate = max_rotate_velocity_;
        // if (dist < 1) {
        //     int wi = carry_id_ == 0 ? Dispatch::plan2_[id_].
        // }
        // 110 启动，60加速
        const double st = 100, ed = 60;
        // double rate = 1;
        double rate = std::max(0.1, 1 + (fabs(dif_rot) / PI - ed/180.0) * (1 / ((ed - st) / 180.0)));
        // double rate = (1 + (fabs(dif_rot) / PI - 1.0/5) * (-5.0/4)); // 1% 优化
        forward = std::min(max_forward_velocity_ * rate, max_rotate_velocity_ * cir); // not bad solution
        // forward = max_forward_velocity_;

        // forward = std::min(max_forward_velocity_ * std::min(1.0, rate), max_rotate_velocity_ * cir); // not bad solution
        // 180 0
        // 90 4
        // 50 5
        // 30 6
        // if (fabs(dif_rot))
        // double cntv = std::max(GetLinearVelocity(), 1e-6);
        // forward = sqrt(cntv * max_rotate_velocity_ * cir);
        // rotate = cntv / forward * max_rotate_velocity_;
        rotate *= dcmp(dif_rot);
    }
}

void Robot::ToPointTwoPoint(Point a, Point b, double& forward, double& rotate, int frame_a) {
    double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
    // Log::print("Dist", Geometry::Dist(x0_, y0_, a.x, a.y));
    if (Geometry::Length(a - pos_) < std::max(0.4, Geometry::UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), 0.0)) + (frame_a == INT_MAX ? 2 : 0) && Input::frameID + GetLinearVelocity() / (max_force_ / GetMass()) * 50 < frame_a) {
        Log::print("stop and rotate");
        forward = 0;
        double aim_rot = Geometry::Angle((frame_a == INT_MAX ? a : b)-pos_);
        double dif_rot = AngleReg(aim_rot - orient_);
        if (fabs(dif_rot) <= limit_r) {
            rotate = 0; // 开始角速度减速
        } else {
            rotate = max_rotate_velocity_;
            rotate *= dcmp(dif_rot);
        }
        return;
    }
    Point cnt = pos_;
    double alpha = InterAngle(a - cnt, b - a);
    if (alpha > PI / 2 && Geometry::Length(a - pos_) > 2) {
        a = a + (a - b) / Length(a - b) * -0.4;
        // Log::print("changea", a.x, a.y);
    }
    double dist = Geometry::Length(a - pos_);
    double aim_rot = Geometry::Angle(a - pos_);
    double dif_rot = AngleReg(aim_rot - orient_);
    double cir = Geometry::MinRadius(dist, fabs(dif_rot));
    const double st = 100, ed = 60;
    if (fabs(dif_rot) <= limit_r) {
        rotate = 0; // 开始角速度减速
        forward = max_forward_velocity_;
    } else {
        rotate = max_rotate_velocity_;
        // 110 启动，60加速
        double rate = std::max(0.1, 1 + (fabs(dif_rot) / PI - ed/180.0) * (1 / ((ed - st) / 180.0)));
        // double rate = (1 + (fabs(dif_rot) / PI - 1.0/5) * (-5.0/4)); // 1% 优化
        // double rate = 1;
        forward = std::min(max_forward_velocity_ * rate, max_rotate_velocity_ * cir * 0.85); // not bad solution
        rotate *= dcmp(dif_rot);
    }
    double rate = std::max(0.1, 1 + (fabs(alpha) / PI - ed/180.0) * (1 / ((ed - st) / 180.0)));
    double aim_v = max_forward_velocity_ * rate;
    double slow_dist = UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), aim_v);
    if (dist < slow_dist) {
        // Log::print("slow_dist", dist, slow_dist);
        forward = aim_v;
    }
    if (dist < 0.6) {// 提前角速度开转{
        // Log::print("rotate_angu", dist, slow_dist);
        rotate = max_rotate_velocity_ * dcmp(atan2(b.y - a.y, b.x - a.x));
    }
}

void Robot::AvoidToWall(double &forward, double &rotate) {
    double limit = CalcMaxSlowdownDist();
    /*
    double walld = WayFinding::DistToWall(pos_, GetLinearVelocity() > 0.5 ? Angle(linear_velocity_) : orient_);
    // if (Input::frameID == 117 && id_ == 0) {
    //     Log::print(id_, walld, limit);
    // }

    if (limit >= walld - 0.58) {
        forward = 0;
    }
    */
}

//double Robot::DistToWall(Point p, double orient) {
//    double mind = 100;
//    Vector ori{cos(orient), sin(orient)};
//    const static std::vector<std::pair<Point, double>> wall{
//            {{0,0.40}, 0},
//            {{49.60,0}, PI/2},
//            {{50,49.60}, PI},
//            {{0.40,50}, -PI/2},
//    };
//    for (const auto& [wp, wo] : wall) {
//        Point sec = GetLineIntersection2(p, ori, wp, {cos(wo), sin(wo)});
//        // if (Input::frame)
//        if (Dot(sec - p, ori) > 0) {
//            mind = std::min(mind, Length(sec - p));
//            // if (Input::frameID == 1069 && id_ == 0) {
//            //     Log::print(id_, Length(sec - p), ori.x, ori.y);
//            //     Log::print(p.x, p.y);
//            // }
//        }
//    }
//    return mind;
//}

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
    return sqrt(0.95 * max_force_ * r / GetMass()); // 0.95 转向时加速度用不完
}

double Robot::CalcTime(const Point& p) {
    // bool P = true;
    double aim_r = Geometry::Angle(p - pos_);
    double dif_r = fabs(AngleReg(aim_r - orient_));
    double dist = Geometry::Length(p - pos_);
    double ans = UniformVariableDist2(max_rot_force_ / GetRotInerta(), dif_r, angular_velocity_, max_rotate_velocity_ * (dif_r > 0 ? 1 : -1));
    double linearV = GetLinearVelocity();
    double cir = std::min(1.5 * linearV / max_forward_velocity_, MinRadius(dist, dif_r));
    // Log::print(dist);
    // if (P) Log::print("T", ans, max_rot_force_ / GetRotInerta(), dif_r, angular_velocity_, max_rotate_velocity_ * dcmp(dif_r));
    dist -= 2 * cir * sin(dif_r / 2);
    // Log::print(dist);
    // double t1 = ans;
    // Log::print(dif_r, cir, angular_velocity_);

    double a = max_force_ / GetMass();
    double ans_up_speed = (max_forward_velocity_ - linearV) / a;
    // Log::print(ans_up_speed, ans);
    ans = 0.97 * std::max(ans, ans_up_speed) + 0.15 * std::min(ans, ans_up_speed);
    // if (P) Log::print(ans);
    dist -= UniformVariableDist(a, GetMaxSpeedOnCir(cir), max_forward_velocity_);
    // Log::print(dist);
    // double t2 = ans;
    // Log::print(dist);
    // Log::print(cir, dif_r);
    ans += (dist + cir * dif_r * 0.7) / max_forward_velocity_; // 这句话对圆周，非常不准确
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
    a.pos_ = p1;
    a.orient_ = Geometry::Angle(p2 - p1);
    a.angular_velocity_ = 0;
    a.linear_velocity_ = Vector{cos(a.orient_), sin(a.orient_)} * max_forward_velocity_;
    a.carry_id_ = 1;
    // Log::print(p1.x, p1.y, p2.x, p2.y, a.CalcTime(p2));
    ans += a.CalcTime(p2);
    return ans;
}


double Robot::GetLinearVelocity() {
    return Geometry::Length(linear_velocity_);
}
double Robot::CalcMaxSlowdownDist() {
    return Geometry::UniformVariableDist(max_force_ / GetMaxMass(), GetLinearVelocity(), 0);
}

// void Robot::Robot_Control(double& forward, double& rotate) {
//     Log::print("Robot_Control, id: ", id_, "route: ", route_.size(), "v: ", v.size());
//     Log::print(pos_.x, " ", pos_.y, '\n');
//     if (route_.empty()) {
//         if (v.empty() || !WayFinding::GetOfflineRoute(v.front()[1], pos_, last_point_, v.front()[0], v.front()[2], route_))
//             Log::print("find route failed");
//         Log::print("routes_size: ", route_.size());
//         for(const auto& point: route_) {
//             Log::print(point.x, point.y);
//         }
//     }
//     Point robot_pos = pos_;
//     while (route_.size() && Geometry::Length(robot_pos - route_.front()) < 0.1) { // 机器人到达某个点，则删掉。
//         // Log::print("reach");
//         route_.erase(begin(route_)); // 不能直接删除，可能需要回滚
//         if (route_.empty()) {
//             last_point_ = WayFinding::workbench_extern_id[v.front()[0]][v.front()[2]] + Input::robot_num_;
//             Log::print("LastPoint: ", v.front()[0], " ", v.front()[2], " ", last_point_);
//             v.erase(begin(v));
//         }
//     }
//     //上下左右半径0.25
//     //斜0.35
//     //

//     if (route_.size()) {
//         ToPoint_1(route_.front(), forward, rotate);
//         // Log::print("frame: ", Input::frameID ,"id: ", id_, "forward: ",forward, "rotate: ",rotate, "pos.x: ", pos_.x, "pos.y: ", pos_.y, "to.x: ", route_.front().x, "to.y: ", route_.front().y);
//         return;
//     }
//     forward = 0; rotate = 0;
// }