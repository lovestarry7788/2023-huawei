//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"
#include "geometry.h"
#include "log.h"
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
        angular_velocity_(angular_velocity), linear_velocity_x_(linear_velocity_x), linear_velocity_y_(linear_velocity_y), orient_(orient), x0_(x0), y0_(y0){}

// 方案2：控制通过转向圆周控制速度，除了圆周不够，都不减速
void Robot::ToPoint_3(double dx, double dy, double& forward, double& rotate) {
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

// 方案1：从当前点到某点，到达时速度为0（防止走过头，更容易控制）。
void Robot::ToPoint_1(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = AngleReg(orient_ - aim_rot);

    // static int frame = 0;
    // Log::print("frame", ++frame);
    if (fabs(dif_rot) > max_orient_diff_) {
        // 2 max
        // 2.6
        if (fabs(dif_rot) > 1.5) forward = 0; // 角度太大就停下再转，防止绕圈圈
        else forward = max_forward_velocity_;
        // else forward = max_forward_velocity_ * std::max(0.0, 3.5-1 / 0.6 * fabs(dif_rot));
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
#include "dispatch.h"

// 方案2：控制通过转向圆周控制速度，除了圆周不够，都不减速
void Robot::ToPoint(double dx, double dy, double& forward, double& rotate) {
    double aim_rot = atan2(dy-y0_, dx-x0_);
    double dif_rot = AngleReg(aim_rot - orient_);
    double dist = Geometry::Dist(x0_, y0_, dx, dy);

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
        forward = std::min(max_forward_velocity_ * rate, max_rotate_velocity_ * cir * 0.7); // not bad solution
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
    if (Geometry::Dist(x0_, y0_, a.x, a.y) < std::max(0.4, Geometry::UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), 0.0)) + (frame_a == INT_MAX ? 2 : 0) && Input::frameID + GetLinearVelocity() / (max_force_ / GetMass()) * 50 < frame_a) {
        // Log::print("stop and rotate");
        forward = 0;
        double aim_rot = atan2((frame_a == INT_MAX ? a.y : b.y)-y0_, (frame_a == INT_MAX ? a.x : b.x)-x0_);
        double dif_rot = AngleReg(aim_rot - orient_);
        if (fabs(dif_rot) <= limit_r) {
            rotate = 0; // 开始角速度减速
        } else {
            rotate = max_rotate_velocity_;
            rotate *= dcmp(dif_rot);
        }
        return;
    }
    Point cnt{x0_, y0_};
    double alpha = InterAngle(a - cnt, b - a);
    if (alpha > PI / 2 && Geometry::Dist(x0_, y0_, a.x, a.y) > 2) {
        a = a + (a - b) / Length(a - b) * -0.4;
        // Log::print("changea", a.x, a.y);
    }
    double dx = a.x, dy = a.y;
    double dist = Geometry::Dist(x0_, y0_, dx, dy);
    double aim_rot = atan2(dy-y0_, dx-x0_);
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
        forward = std::min(max_forward_velocity_ * rate, max_rotate_velocity_ * cir); // not bad solution
        rotate *= dcmp(dif_rot);
    }
    double rate = std::max(0.1, 1 + (fabs(alpha) / PI - ed/180.0) * (1 / ((ed - st) / 180.0)));
    double aim_v = max_forward_velocity_ * rate;
    double slow_dist = UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), aim_v);
    if (dist < slow_dist) {
        // Log::print("slow_dist", dist, slow_dist);
        forward = aim_v;
    }
    if (dist < 0.8) // 提前角速度开转
        rotate = max_rotate_velocity_ * dcmp(atan2(b.y - a.y, b.x - a.x));
}
// void Robot::ToPointTwoPoint(Point a, Point b, double& forward, double& rotate) {
//     Point cnt{x0_, y0_};
//     double alpha = InterAngle(a - cnt, b - a);
//     Vector p1 = cnt - a, p2 = b - a;
//     Vector aim = p1 / Length(p1) + p2 / Length(p2);
//     Point mid = (cnt + a) / 2;
//     Vector ca_k = cnt - a;
//     std::swap(ca_k.x, ca_k.y); ca_k.x = -ca_k.x;
//     Point center = GetLineIntersection2(a, aim, mid, ca_k);
//     double R = Length(center - a);
//     // 1-> 0.5
//     // 1/3 -> 1
//     double rate = std::min(1.0, 1 + (fabs(alpha) / PI - 1.0/3) * (-3.0/4));
//     double r_limit = max_forward_velocity_ / max_rotate_velocity_ * rate;
//     double r = std::min(r_limit, R);
//     // double r = R;
//     Log::print("A", R, r, alpha, center.x, center.y); // r不应该迅速下降，
//     if (R > r_limit) {
//         // 向P3点移动
//         center = a + (center - a) / Length(center - a) * r;
//         Vector u = center - cnt;
//         double ang = asin(r / Length(u));
//         Point p3 = Rotate(u, ang);
//         p3 = cnt + p3 / Length(p3) * sqrt(Length(u) * Length(u) - r * r);
//         Point p4 = Rotate(u, -ang);
//         p4 = cnt + p4 / Length(p4) * sqrt(Length(u) * Length(u) - r * r);
//         if (Length(p3 - a) > Length(p4 - a)) {
//             std::swap(p3, p4);
//         }
//         double aim_rot = atan2(p3.y - cnt.y, p3.x - cnt.x);
//         double dif_rot = AngleReg(aim_rot - orient_);
//         double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);

//         if (fabs(dif_rot) + 0.0628 > limit_r)
//             rotate = std::min(max_rotate_velocity_, fabs(dif_rot) * 50) * dcmp(dif_rot);
//             // rotate = max_rotate_velocity_ * dcmp(dif_rot);
//         else
//             rotate = 0;
//         forward = max_forward_velocity_;
//         // double rate = 1;
//         // double slow_down = UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), on_cir_v);
//         // if (Length(p3 - cnt) < slow_down) forward = on_cir_v;
//         // else forward = max_forward_velocity_;
//         Log::print("P", aim_rot, orient_, atan2(linear_velocity_y_, linear_velocity_x_), dif_rot, limit_r, p3.x, p3.y);
//     } else {
//         ToPoint(a.x, a.y, forward, rotate);
//         // forward = max_rotate_velocity_ * R;
//         // Log::print("T", max_rotate_velocity_, R);
//         // rotate = std::min(max_rotate_velocity_, GetLinearVelocity() / R);
//         // double aim_rot = atan2(a.y - cnt.y, a.x - cnt.x);
//         // double dif_rot = AngleReg(aim_rot - orient_);
//         // rotate *= dcmp(dif_rot);
//     }
// }
// void Robot::ToPointTwoPoint(Point a, Point b, double& forward, double& rotate) {
//     Point cnt{x0_, y0_};
//     double alpha = InterAngle(cnt - a, b - a);
//     bool P = id_ == 2;
//     double dx = a.x, dy = a.y;
//     double aim_rot = atan2(dy-y0_, dx-x0_);
//     double dif_rot = AngleReg(aim_rot - orient_);
//     double dist = Geometry::Dist(x0_, y0_, dx, dy);

//     double cir = Geometry::MinRadius(dist, fabs(dif_rot));
//     // if (fabs(dif_rot) > 0.5 && cir < 2 && dist < 3)
//     //     cir = Geometry::MinRadius2(dx-x0_, dy-y0_, fabs(aim_rot));
//     // forward = GetMaxSpeedOnCir(cir);
//     // forward = std::min(forward, max_forward_velocity_);
//     double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
//     // if (Input::frameID > 230 && Input::frameID < 260 && id_ == 3)
//     //     Log::print("ToPoint", angular_velocity_, limit_r, dif_rot, dx, dy, x0_, y0_);
//     // double linearV = GetLinearVelocity();
//     // 速度不匹配？相差角度太大？cir变化太大？没有好的表示形式，更没有好解决方法。不碰撞自己转，乱调参数则出问题。
//     if (fabs(dif_rot) < limit_r) {
//         rotate = 0; // 开始角速度减速
//         forward = max_forward_velocity_;
//     }
//     else {
//         rotate = max_rotate_velocity_;
//         forward = max_forward_velocity_; // not bad solution
//         // double cntv = std::max(GetLinearVelocity(), 1e-6);
//         // forward = sqrt(cntv * max_rotate_velocity_ * cir);
//         // rotate = cntv / forward * max_rotate_velocity_;
//         rotate *= dcmp(dif_rot);
//     }
//     double speed_on_cir = std::min(max_forward_velocity_ * alpha / PI, max_rotate_velocity_ * cir);
//     double slow_dist = Geometry::UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), speed_on_cir);
//     if (dist < slow_dist) forward = speed_on_cir;
//     // cir < 1 仅用于小圈转入情况，dif_rot与PI/2相近，只用于目标点在圆心处。绕圈特征：dif_rot随时间变化，以PI/2为中心，0.5幅度变化。
//     // if (cir < 2.0 && fabs(fabs(dif_rot) - PI/2) < 0.3 && fabs(fabs(dif_rot) - PI/2) > 0.05) { // 调参
//     //     Log::print("ToPoint2", id_, linearV, forward, fabs(fabs(dif_rot) - PI/2), cir);
//     //     rotate *= 0.40*cir; // 调参 1 -> 0.4; 2->0.8 cir * 0.4
//     //     // forward *= 0.5;
//     // }
//     if(P) Log::print(alpha, cir, speed_on_cir);
// }
// void Robot::ToPointTwoPoint(Point a, Point b, double& forward, double& rotate) {
//     const double r = 1;
//     Point cnt = Point{x0_, y0_};
//     Vector p1 = cnt - a, p2 = b - a;
//     double aplha = InterAngle(cnt - a, b - a) / 2;
//     double d = Length(cnt - a), x = sqrt(d * d - 2 * d * r * cos(aplha)), y = sqrt(x * x + r * r);
//     Vector aim = p1 / Length(p1) + p2 / Length(p2);
//     aim = aim * r / Length(aim);
//     Point center = a + aim;

//     Vector u = center - cnt;
//     double ang = asin(r / Length(u));
//     Point p3 = Rotate(u, ang);
//     p3 = cnt + p3 / Length(p3) * sqrt(Length(u) * Length(u) - r * r);
//     if (Length(p3 - a) > Length(p3 - b)) {
//         p3 = Rotate(u, -ang);
//         p3 = cnt + p3 / Length(p3) * sqrt(Length(u) * Length(u) - r * r);
//     }
//     double on_cir_v = r * max_rotate_velocity_;
//     if (x > 0.1) {
//         double aim_rot = atan2(p3.y - cnt.y, p3.x - cnt.x);
//         double dif_rot = AngleReg(aim_rot - orient_);
//         if (fabs(dif_rot) > 0.03)
//             rotate = max_rotate_velocity_ * dcmp(dif_rot);
//         else
//             rotate = 0;
//         double slow_down = UniformVariableDist(max_force_ / GetMass(), GetLinearVelocity(), on_cir_v);
//         if (Length(p3 - cnt) < slow_down) forward = on_cir_v;
//         else forward = max_forward_velocity_;
//         // Log::print(u.x, u.y, p3.x, p3.y, ang, x, forward, rotate);
//     } else {
//         double aim_rot = atan2(a.y - cnt.y, a.x - cnt.x);
//         double dif_rot = AngleReg(aim_rot - orient_);
//         if (fabs(dif_rot) > 0.03)
//             rotate = -max_rotate_velocity_;
//         else
//             rotate = 0;
//         forward = on_cir_v;
//         Log::print("x<");
//     }
//     Log::print(Length(cnt - center) - r);
// }
// TODO：突然的急转弯难预测？本函数检查。
std::vector<Geometry::Point> Robot::ForecastToPoint(double dx, double dy, int forecast_num){
    // bool P = Input::frameID == 926 && (id_ == 2 && fabs(rotate - 0) < 1e-6 && fabs(forward - 0) < 1e-6 || id_ == 3);
    // bool P = false;

    // 假设：加速度总是满
    std::vector<Point> forecast(forecast_num, Point{0,0});
    double linearV = GetLinearVelocity();
    double angularV = angular_velocity_;
    double x0 = x0_, y0 = y0_;
    double orient = orient_;
    // if (P) Log::print("ForecastFixed", x0, y0, orient, linearV, angularV);
    for (int i = 0; i < forecast_num; i++) {

        linearV = std::max(1e-8, linearV);
        angularV = std::max(1e-8, fabs(angularV)) * (angularV > 0 ? 1 : -1);

        double cir = linearV / fabs(angularV);
        Point center = {x0, y0};
        Vector mov = Vector{-sin(orient), cos(orient)} * cir;
        center = center + mov;
        forecast[i] = center - Rotate(mov, 1 / 50.0 * linearV / cir);
        // if (P) Log::print(forecast_[ri][i].x, forecast_[ri][i].y);

        double forward, rotate;
        ToPoint(dx, dy, forward, rotate);
        // update
        if (fabs(linearV - forward) > 1e-3) {
            double add = (forward - linearV > 0 ? 1 : -1) * 1 / 50.0 * max_force_ / GetMass();
            if ((linearV - forward) * (linearV + add - forward) < 0) {
                linearV = 0;
            } else {
                linearV += add;
                linearV = std::min(linearV, max_forward_velocity_);
            }
        }
        if (fabs(angularV - rotate) > 1e-3) {
            double add = (rotate - angularV > 0 ? 1 : -1) * 1 / 50.0 * max_rot_force_ / GetRotInerta();
            if ((angularV - rotate) * (angularV + add - rotate) < 0) {
                angularV = 0;
            } else {
                angularV += add;
                angularV = std::min(max_rotate_velocity_, fabs(angularV)) * (angularV > 0 ? 1 : -1);
            }
        }

        x0 = forecast[i].x;
        y0 = forecast[i].y;
        orient += angularV * 1 / 50.0;
        // if (P) Log::print(x0, y0, cir, orient, center.x, center.y, linearV, angularV);
    }
    return forecast;
}
// std::vector<Geometry::Point> Robot::ForecastToPoint(double dx, double dy, int forecast_num) {
//     double aim_rot = atan2(dy-y0_, dx-x0_);
//     double dif_rot = AngleReg(aim_rot - orient_);
//     double dist = Geometry::Dist(x0_, y0_, dx, dy);
//     double cir = Geometry::MinRadius(dist, fabs(dif_rot));
//     double limit_r = Geometry::UniformVariableDist(max_rot_force_ / GetRotInerta(), angular_velocity_, 0.0);
//     Point dif0p = {x0_, y0_};

//     bool P = Input::frameID == 244 && id_ == 0;
//     std::vector<Point> forecast;
//     double lstV = GetLinearVelocity();
//     double rotate, forward;
//     if (fabs(dif_rot) < limit_r) {
//         rotate = 0; // 开始角速度减速
//         forward = max_forward_velocity_;
//     }
//     else {
//         rotate = max_rotate_velocity_;
//         forward = std::min(max_forward_velocity_, max_rotate_velocity_ * cir); // not bad solution
//         rotate *= dcmp(dif_rot);
//         Point center = {x0_, y0_};
//         Vector mov = Vector{-sin(orient_), cos(orient_)} * cir * -dcmp(rotate);
//         center = center - mov;
//         double linearV = GetLinearVelocity();
//         for (int ti = 1; ti <= int(dif_rot / rotate * 50); ti++) {
//             if (fabs(linearV - forward) > 1e-3) {
//                 double add = (forward - linearV > 0 ? 1 : -1) * 1 / 50.0 * max_force_ / GetMass();
//                 if ((linearV - forward) * (linearV + add - forward) < 0) {
//                     linearV = 0;
//                 } else {
//                     linearV += add;
//                     linearV = std::min(linearV, max_forward_velocity_);
//                 }
//             }
//             forecast.push_back(center + Rotate(mov, ti / 50.0 * GetLinearVelocity() / cir)); // 这里rotate不好，用forward代替，因为cir突变
//             // if (P) Log::print(forecast.back().x, forecast.back().y);
//         }
//         lstV = forward;
//         if (P) {
//             Log::print(aim_rot, orient_, dy, y0_, dx, x0_);
//             Log::print(dif_rot, cir, mov.x, mov.y, center.x, center.y, rotate);
//             auto t = center + mov;
//             Log::print(t.x, t.y);
//             t = center + Rotate(mov, 1 / 50.0 * rotate);
//             Log::print(t.x, t.y);
//             t = center + Rotate(mov, 2 / 50.0 * rotate);
//             Log::print(t.x, t.y);
//         }
//     }
//     Point lst_p = forecast.size() ? forecast.back() : Point{x0_, y0_};
//     double aim = atan2(dy-lst_p.y, dx-lst_p.x);
//     Vector aim_e = Vector{cos(aim), sin(aim)};
//     while (forecast.size() < forecast_num) {
//         lst_p = lst_p + aim_e * lstV * 1 / 50.0;
//         forecast.push_back(lst_p);
//         lstV += max_force_ / GetMass() * 1 / 50.0;
//         lstV = std::min(lstV, max_forward_velocity_);
//     }
//     if (P) {
//         Log::print("ForecastToPoint", id_);
//         for (auto i : forecast) Log::print(i.x, i.y);
//     }
//     return forecast;
// }

std::vector<Geometry::Point> Robot::ForecastFixed(double forward, double rotate, int forecast_num) {
    bool P = Input::frameID == 926 && (id_ == 2 && fabs(rotate - 0) < 1e-6 && fabs(forward - 0) < 1e-6 || id_ == 3);

    if (P) Log::print("Forecast", id_, forward, rotate);
    // 假设：加速度总是满
    std::vector<Point> forecast(forecast_num, Point{0,0});
    double linearV = GetLinearVelocity();
    double angularV = angular_velocity_;
    double x0 = x0_, y0 = y0_;
    double orient = orient_;
    if (P) Log::print("ForecastFixed", x0, y0, orient, linearV, angularV);
    for (int i = 0; i < forecast_num; i++) {

        linearV = std::max(1e-8, linearV);
        angularV = std::max(1e-8, fabs(angularV)) * (angularV > 0 ? 1 : -1);

        double cir = linearV / fabs(angularV);
        Point center = {x0, y0};
        Vector mov = Vector{-sin(orient), cos(orient)} * cir;
        center = center + mov;
        forecast[i] = center - Rotate(mov, 1 / 50.0 * linearV / cir);
        // if (P) Log::print(forecast_[ri][i].x, forecast_[ri][i].y);

        // update
        if (fabs(linearV - forward) > 1e-3) {
            double add = (forward - linearV > 0 ? 1 : -1) * 1 / 50.0 * max_force_ / GetMass();
            if ((linearV - forward) * (linearV + add - forward) < 0) {
                linearV = 0;
            } else {
                linearV += add;
                linearV = std::min(linearV, max_forward_velocity_);
            }
        }
        if (fabs(angularV - rotate) > 1e-3) {
            double add = (rotate - angularV > 0 ? 1 : -1) * 1 / 50.0 * max_rot_force_ / GetRotInerta();
            if ((angularV - rotate) * (angularV + add - rotate) < 0) {
                angularV = 0;
            } else {
                angularV += add;
                angularV = std::min(max_rotate_velocity_, fabs(angularV)) * (angularV > 0 ? 1 : -1);
            }
        }

        x0 = forecast[i].x;
        y0 = forecast[i].y;
        orient += angularV * 1 / 50.0;
        if (P) Log::print(x0, y0, cir, orient, center.x, center.y, linearV, angularV);
    }
    return forecast;
}

void Robot::AvoidToWall(double &forward, double &rotate) {
    double limit = CalcMaxSlowdownDist();
    double walld = DistToWall({x0_, y0_}, GetLinearVelocity() > 0.5 ? atan2(linear_velocity_y_, linear_velocity_x_) : orient_);
    // if (Input::frameID == 117 && id_ == 0) {
    //     Log::print(id_, walld, limit);
    // }

    if (limit >= walld - 0.58) {
        forward = 0;
    }
}

double Robot::DistToWall(Point p, double orient) {
    double mind = 100;
    Vector ori{cos(orient), sin(orient)};
    const static std::vector<std::pair<Point, double>> wall{
            {{0,0.40}, 0},
            {{49.60,0}, PI/2},
            {{50,49.60}, PI},
            {{0.40,50}, -PI/2},
    };
    for (const auto& [wp, wo] : wall) {
        Point sec = GetLineIntersection2(p, ori, wp, {cos(wo), sin(wo)});
        // if (Input::frame)
        if (Dot(sec - p, ori) > 0) {
            mind = std::min(mind, Length(sec - p));
            // if (Input::frameID == 1069 && id_ == 0) {
            //     Log::print(id_, Length(sec - p), ori.x, ori.y);
            //     Log::print(p.x, p.y);
            // }
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
    return sqrt(0.95 * max_force_ * r / GetMass()); // 0.95 转向时加速度用不完
}

double Robot::CalcTime(const Point& p) {
    // bool P = true;
    double aim_r = atan2(p.y - y0_, p.x - x0_);
    double dif_r = fabs(AngleReg(aim_r - orient_));
    double dist = Dist(p.x, p.y, x0_, y0_);
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
    a.x0_ = p1.x;
    a.y0_ = p1.y;
    a.orient_ = atan2(p2.y - p1.y, p2.x - p1.x);
    a.angular_velocity_ = 0;
    a.linear_velocity_x_ = max_forward_velocity_ * cos(a.orient_);
    a.linear_velocity_y_ = max_forward_velocity_ * sin(a.orient_);
    a.carry_id_ = 1;
    // Log::print(p1.x, p1.y, p2.x, p2.y, a.CalcTime(p2));
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