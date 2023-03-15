#include "dispatch.h"
#include "input.h"
#include "output.h"

std::vector<std::set<Point>> forecast_;

std::vector<Plan> plan_;

std::function<void(int)> robotPlanEnded_;

void init(std::function<void(int)> plan_ed, int robot_num);

// 被robotPlanEnded_调用。更新plan，否则默认继承上帧plan
void UpdatePlan(int robot_id, Plan plan) {
    ManagePlan(robot_id, plan);
    plan_[robot_id] = plan;
}

// 处理计划，外部call
void ManagePlan() {
    for (int i = 0; i < plan_.size(); i++)
        ManagePlan(i, plan_[i]);
}

// 是否现在可以完成，可以则调用 robotPlanEnded_
void ManagePlan(int robot_id, const Plan& plan) {

}

// 输出行走
void ControlWalk() {
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        auto robot = Input::robot[ri];
        int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
        if (wi == -1) continue;
        double forward, rotate;
        Input::robot[ri]->ToPoint(forward, rotate);
        OutPut::Forward(ri, forward);
        OutPut::Rotate(ri, rotate);
    }
}
