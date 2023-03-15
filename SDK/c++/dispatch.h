#ifndef HW2023_DISPATCH_H
#define HW2023_DISPATCH_H

#include "input.h"
#include "geometry.h"
#include "output.h"
#include <set>

// 负责运送机器人过程，包含避障
// 先购买/出售，再addplan，再调用FrameEnd交给dispatch处理topoint
namespace Dispatch {
    using namespace Geometry;
    extern std::vector<std::set<Point>> forecast_;
    struct Plan {
        // int robot_id;
        int buy_workbench = -1, sell_workbench = -1; // -1为已完成，否则为工作台id
    };
    extern std::vector<Plan> plan_;
    
    // 完成Plan时调用委托。单独(延迟)规划用
    extern std::function<void(int)> robotPlanEnded_;
    
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
    void ManagePlan(int robot_id, const Plan& plan);

    // 输出行走
    void ControlWalk();

    // 外部主循环中，manageplan(), controlwalk()。决策中，如果是完成任务再规划，则robotPlanEnded_调用addplan；如果要每时每刻重新单独规划，则在manageplan前调用一遍robotPlanEnded_；如果每时每刻整体规划，则直接调用UpdatePlan；如果固定时间规划一次，则固定时间调用
}

#endif