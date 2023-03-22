#ifndef HW2023_DISPATCH_H
#define HW2023_DISPATCH_H

#include "input.h"
#include "geometry.h"
#include "output.h"
#include <vector>

// 负责运送机器人过程，包含避障
// 先购买/出售，再addplan，再调用FrameEnd交给dispatch处理topoint
namespace Dispatch {
    using namespace Geometry;
    // extern std::vector<std::vector<Geometry::Point>> forecast_;
    constexpr int forecast_num_ = 75;
    constexpr int forecast_per_ = 5;
    constexpr double collide_dist_ = 1.7;
    constexpr double collide_time_elemit_ = 0.02; // collide(dist, t) = dist < collide_dist_ - t * collide_time_elemit_;
    struct Plan {
        // int robot_id;
        int mat_id = 0;
        int buy_workbench = -1, sell_workbench = -1; // -1为已完成，否则为工作台id
    };
    extern std::vector<Plan> plan_;
    extern std::vector<std::pair<double, double>> movement_;
    // 工作台占用情况
    struct Occupy {
        bool buy_occupy = 0;
        int sell_occupy = 0; // >>i&1 则i物品被占用
    };
    extern std::vector<Occupy> occupy_;
    extern bool avoidCollide;

    // 完成Plan时调用委托。单独(延迟)规划用
    extern void (*RobotReplan_)(int);
    // extern std::function<void(int,int)> Dispatch::RobotBuy_;
    // extern std::function<void(int,int,int)> Dispatch::RobotSell_;

    void init(void (*RobotReplan)(int), int robot_num, int workbench_num);
    // void init(std::function<void(int)> RobotReplan, std::function<void(int,int)> RobotBuy, std::function<void(int,int,int)> RobotSell, int robot_num);

    void UpdateAll();

    void UpdateCompleted();

    // 被robotReplan_调用。更新plan，否则默认继承上帧plan
    void UpdatePlan(int robot_id, Plan plan);

    // 处理计划，外部call
    void ManagePlan();

    // 是否现在可以完成，可以则调用 robotReplan_
    void ManagePlan(int robot_id, Plan& plan);

    // 输出行走
    void ControlWalk();

    void AvoidCollide();

    double ForecastCollide(const std::vector<Point>& a, const std::vector<Point>& b, double mx_dist);
    // 外部主循环中，manageplan(), controlwalk()。决策中，如果是完成任务再规划，则robotReplan_调用addplan；如果要每时每刻重新单独规划，则在manageplan前调用一遍robotReplan_；如果每时每刻整体规划，则直接调用UpdatePlan；如果固定时间规划一次，则固定时间调用
}

#endif