#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "log.h"
#include "geometry.h"

using namespace Dispatch;
using namespace Geometry;
std::vector<std::vector<Geometry::Point>> Dispatch::forecast_;
std::vector<std::pair<double, double>> Dispatch::movement_;

std::vector<Plan> Dispatch::plan_;

std::vector<Occupy> Dispatch::occupy_;

// std::function<void(int)> Dispatch::RobotReplan_;
void (*Dispatch::RobotReplan_)(int);

void Dispatch::init(void (*RobotReplan)(int), int robot_num, int workbench_num) {
    RobotReplan_ = RobotReplan;
    plan_.resize(robot_num);
    occupy_.resize(workbench_num);
}

// 被robotReplan_调用。更新plan，否则默认继承上帧plan
void Dispatch::UpdatePlan(int robot_id, Plan plan) {
    // occpuy plan，可只有buy or sell
    // Log::print("UpdatePlan", robot_id, plan.buy_workbench, plan.sell_workbench);
    if (plan.buy_workbench != -1)
        occupy_[plan.buy_workbench].buy_occupy = true;
    if (plan.sell_workbench != -1)
        occupy_[plan.sell_workbench].sell_occupy |= (1<<plan.mat_id);
    // ManagePlan(robot_id, plan);
    plan_[robot_id] = plan;
}

// // clear unoccupy_queue_
// void Dispatch::ClearUnoccupy() {
//     for (auto uno : unoccupy_queue_) {
//         if (uno.buy_workbench != -1) {
//             Unoccupy_buy(uno.buy_workbench, uno.mat_id);
//         }
//         if (uno.sell_workbench != -1){
//             Unoccupy_sell(uno.sell_workbench, uno.mat_id);
//         }
//     }
//     unoccupy_queue_.clear();
// }

/*
UpdateAll  / Update-1
ManagePlan(ClearUnoccupy) 完成则改成-1。Replan必须在下帧，本函数无资格
*/

void Dispatch::UpdateAll() {
    // unoccupy all not -1 plan
    for (size_t i = 0; i < plan_.size(); i++) {
        if (plan_[i].buy_workbench != -1)   
            occupy_[plan_[i].buy_workbench].buy_occupy = false;
        if (plan_[i].sell_workbench != -1)   
            occupy_[plan_[i].sell_workbench].sell_occupy &= 
                ((unsigned)1 << 31) - 1 - (1<<plan_[i].mat_id);
        RobotReplan_(i);// 手上有东西则更新sell
    }
}
void Dispatch::UpdateCompleted() {
    for (size_t i = 0; i < plan_.size(); i++) {
        auto plan = plan_[i];
        if (plan.buy_workbench == -1 && plan.sell_workbench == -1 && RobotReplan_) {
            RobotReplan_(i);
        }
    }
}
// 处理计划，外部call
void Dispatch::ManagePlan() {
    for (int i = 0; i < plan_.size(); i++)
        ManagePlan(i, plan_[i]);
}

// 是否现在可以完成，可以则调用 RobotReplan_
void Dispatch::ManagePlan(int robot_id, Plan& plan) {
    // Log::print("ManagePlan", robot_id, plan.buy_workbench, plan.sell_workbench);

    auto robot = Input::robot[robot_id];
    int wi = robot->carry_id_ == 0 ? plan.buy_workbench : plan.sell_workbench;
    if (robot->workbench_ == wi && wi != -1) { // 距离可以进行买卖
        if (robot->carry_id_ == 0 && Input::workbench[wi]->product_status_) {
            Output::Buy(robot_id);
            occupy_[wi].buy_occupy = false;
            // Plan out = plan; out.sell_workbench = -1;
            // unoccupy_queue_.push(out);
            plan.buy_workbench = -1;
        } else if (robot->carry_id_ != 0) {
            Output::Sell(robot_id);
            occupy_[wi].sell_occupy &= ((unsigned)1 << 31) - 1 - (1<<robot->carry_id_);
            // Plan out = plan; out.buy_workbench = -1;
            // unoccupy_queue_.push(out);
            plan.sell_workbench = -1;
        } 
    }
}

// 输出行走
void Dispatch::ControlWalk() {
    movement_.resize(Input::robot_num_);
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        auto robot = Input::robot[ri];
        int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
        // Log::print("ControlWalk", ri, plan_[ri].buy_workbench, plan_[ri].sell_workbench);
        if (wi == -1) continue;
        double& forward = movement_[ri].first;
        double& rotate = movement_[ri].second;
        robot->ToPoint(Input::workbench[wi]->x0_, Input::workbench[wi]->y0_, forward, rotate);
        robot -> AvoidToWall(forward, rotate);
        // Log::print(ri, forward, rotate);
        // double v = robot->GetLinearVelocity();
        // if (fabs(v - forward) > 1 && robot->on_cir < 2) rotate /= 4;
//        double limit = robot->CalcSlowdownDist();
//        double walld = DistToWall({robot->x0_, robot->y0_}, robot->orient_);
//        Log::print(ri, limit, walld, robot->GetLinearVelocity(), robot->GetMass());
//        if (limit >= walld - 1.1) {
//            forward = 0;
//        }
        // Log::print("ControlWalk", ri, forward, rotate);
        // if (fabs(forward - invalid) > 1e-5) Output::Forward(ri, forward);
        // if (fabs(rotate - invalid) > 1e-5) Output::Rotate(ri, rotate);
    }
    // Collide();
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        double& forward = movement_[ri].first;
        double& rotate = movement_[ri].second;

        Log::print(ri, forward, rotate);
        Output::Forward(ri, forward);
        Output::Rotate(ri, rotate);
    }
}

void Dispatch::Collide() {
    static std::vector<std::vector<double>> lstang;
    lstang.resize(Input::robot_num_);
    forecast_.resize(Input::robot_num_, std::vector<Point>(forecast_num_, {0, 0}));
    for (int ri = 0; ri < Input::robot_num_; ri++) {
        auto robot = Input::robot[ri];
        bool P = Input::frameID >= 376 && Input::frameID <= 380 && (ri == 2 || ri == 1);
        
        lstang[ri].push_back(robot->angular_velocity_);
        if (lstang[ri].size() > 5) lstang[ri].erase(begin(lstang[ri]));
        double angularV = 0;
        for (auto i: lstang[ri]) angularV += i;
        angularV /= lstang[ri].size();

        if (P) Log::print(angularV);
        if (fabs(angularV) < 1e-8) angularV = 1e-8 * (angularV > 0 ? 1 : -1);
        double linearV = std::max(1e-8, Input::robot[ri]->GetLinearVelocity());
        double cir = linearV / angularV;
        Point center = {robot->x0_, robot->y0_};
        Vector mov = Vector{-sin(robot->orient_), cos(robot->orient_)} * cir;
        center = center + mov;
        if (P) Log::print(cir, angularV, center.x, center.y);
        for (int i = 0; i < forecast_num_; i++) {
            forecast_[ri][i] = center - mov * cos(i / 50.0 * linearV / cir);
            if (P) Log::print(forecast_[ri][i].x, forecast_[ri][i].y);
        }
    }
    std::vector<int> change(Input::robot_num_, 0);
    // &1减速，&2转向
    for (int ri = 0; ri < Input::robot_num_; ri++)
        for (int rj = ri+1; rj < Input::robot_num_; rj++) {
            for (int t = 0; t < forecast_num_; t++) {
                double dist = Length(forecast_[ri][t] - forecast_[rj][t]);
                if (dist > 1.5 + t * 0.03) continue; // 越小对路线估计要求越高，越大越浪费时间
                int c1 = rj, c2 = ri;
                auto roboti = Input::robot[ri];
                auto robotj = Input::robot[rj];

                if (roboti->carry_id_ < robotj->carry_id_ || roboti->carry_id_ == robotj->carry_id_ && ri < rj) std::swap(c1, c2);
                // c2重要
                // change[c2] |= 0b10; // 重要的转
                // change[c1] |= 0b01; // 不重要的减速
                if (fabs(AngleReg(roboti->orient_ - robotj->orient_ - PI)) < 0.3)
                    change[c1] |= 0b10; // 转向
                else change[c1] |= 0b01;
                Log::print("colide", ri, rj, c1, change[c1], Input::robot[c1]->angular_velocity_);
                break;
            }
    }
    for (int ri = 0; ri < Input::robot_num_; ri++) {
        // auto roboti = Input::robot[ri];
        // if (change[ri]) 
        if (change[ri]>>0&1) {
            if (Input::robot[ri]->GetLinearVelocity() >3)
                movement_[ri].first = Input::robot[ri]->GetLinearVelocity();
            else 
                movement_[ri].first = 6;
        }
        if (change[ri]>>1&1) {
            movement_[ri].second = PI * (Input::robot[ri]->angular_velocity_ > 0 ? 1 : -1) ;
            // movement_[ri].first += 1;
        }
        
    }
}
