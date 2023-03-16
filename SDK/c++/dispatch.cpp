#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "log.h"

using namespace Dispatch;

std::vector<std::set<Geometry::Point>> Dispatch::forecast_;

std::vector<Plan> Dispatch::plan_;

// std::function<void(int)> Dispatch::RobotReplan_;
void (*Dispatch::RobotReplan_)(int);
// std::function<void(int,int)> Dispatch::RobotBuy_;
// std::function<void(int,int,int)> Dispatch::RobotSell_;


void Dispatch::init(void (*RobotReplan)(int), int robot_num) {
    RobotReplan_ = RobotReplan;
    plan_.resize(robot_num);
}
// void init(std::function<void(int)> RobotReplan, std::function<void(int,int)> RobotBuy, std::function<void(int,int,int)> RobotSell, int robot_num) {
//     RobotReplan_ = RobotReplan;
//     RobotBuy_ = RobotBuy;
//     RobotSell_ = RobotSell;
//     plan_.resize(robot_num);
// }

// 被robotReplan_调用。更新plan，否则默认继承上帧plan
void Dispatch::UpdatePlan(int robot_id, Plan plan) {
    Log::print("UpdatePlan", robot_id, plan.buy_workbench, plan.sell_workbench);
    ManagePlan(robot_id, plan);
    plan_[robot_id] = plan;
}

// 处理计划，外部call
void Dispatch::ManagePlan() {
    for (int i = 0; i < plan_.size(); i++)
        ManagePlan(i, plan_[i]);
}

// 是否现在可以完成，可以则调用 RobotReplan_
void Dispatch::ManagePlan(int robot_id, Plan& plan) {
    // Log::print("ManagePlan", robot_id, plan.buy_workbench, plan.sell_workbench);
    if (((plan.buy_workbench == -1 && plan.sell_workbench == -1) || plan.update) && RobotReplan_) {
        plan.update = false;
        RobotReplan_(robot_id);
        return;
    }
    auto robot = Input::robot[robot_id];
    int wi = robot->carry_id_ == 0 ? plan.buy_workbench : plan.sell_workbench;
    if (robot->workbench_ == wi && wi != -1) { // 距离可以进行买卖
        if (robot->carry_id_ == 0 && Input::workbench[wi]->product_status_) {
            Output::Buy(robot_id);
            // wi = -1;
        } else if (robot->carry_id_ != 0) {
            Output::Sell(robot_id);
            // robot->carry_id_= 0; // 必须本地修改好，因为再规划要用到。
            // 统一下一帧规划。否则要修改的数据太多了。
            // wi = -1;
            plan.update = true;
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
        if (fabs(forward - invalid) > 1e-5) Output::Forward(ri, forward);
        if (fabs(rotate - invalid) > 1e-5) Output::Rotate(ri, rotate);
    }
}
