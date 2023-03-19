#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "log.h"
#include "geometry.h"

using namespace Dispatch;
using namespace Geometry;
std::vector<std::set<Geometry::Point>> Dispatch::forecast_;

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
    ManagePlan(robot_id, plan);
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
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        auto robot = Input::robot[ri];
        int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
        // Log::print("ControlWalk", ri, plan_[ri].buy_workbench, plan_[ri].sell_workbench);
        if (wi == -1) continue;
        const double invalid = -100;
        double forward = invalid, rotate = invalid;
        Input::robot[ri]->ToPoint(Input::workbench[wi]->x0_, Input::workbench[wi]->y0_, forward, rotate);
        Input::robot[ri] -> AvoidToWall(forward, rotate);
//        double limit = robot->CalcSlowdownDist();
//        double walld = DistToWall({robot->x0_, robot->y0_}, robot->orient_);
//        Log::print(ri, limit, walld, robot->GetLinearVelocity(), robot->GetMass());
//        if (limit >= walld - 1.1) {
//            forward = 0;
//        }
        Log::print("ControlWalk", ri, forward, rotate);
        if (fabs(forward - invalid) > 1e-5) Output::Forward(ri, forward);
        if (fabs(rotate - invalid) > 1e-5) Output::Rotate(ri, rotate);
    }
}
