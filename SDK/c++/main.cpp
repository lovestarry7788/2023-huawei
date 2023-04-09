//
// Created by 刘智杰 on 2023/3/29.
//

#include "log.h"
#include "geometry.h"
#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "wayfinding.h"
#include "simulator.h"
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
#include <ctime>


// 手玩方法，呆滞数据
namespace Solution4 {
    // 走路略快，有些许错配
    std::vector<std::vector<std::pair<int,int>>> route =
    {
        {
            {18, 12},
            {11, 12},
            {18, 12},
            {14, 10},
            {11, 10},
            // {}
        },
        {
            {14, 13},
            {15, 13},
            {14, 13},
            
        },
        {
            {16, 20},
            {22, 20},
            {16, 20},
            {22, 20},
        },
        {
            {19, 20},
        },
    };
    using namespace Input;
    Dispatch::Plan RobotReplan(int robot_id) {
        Dispatch::Plan bst; bst.buy_workbench = bst.sell_workbench = -1;
        if (route[robot_id].size()) {
            auto p = route[robot_id].front();
            bst.buy_workbench = p.first;
            bst.sell_workbench = p.second;
            route[robot_id].erase(begin(route[robot_id]));
        }
        Dispatch::Plan bst2; bst2.buy_workbench = bst2.sell_workbench = -1;
        if (route[robot_id].size()) {
            auto p = route[robot_id].front();
            bst2.buy_workbench = p.first;
            bst2.sell_workbench = p.second;
        }
//        Log::print("UpdatePlan", robot_id, workbench[bst.buy_workbench]->type_id_, workbench[bst.sell_workbench]->type_id_);
        return bst;
        // Dispatch::UpdatePlan(robot_id, bst);
        // Dispatch::plan2_[robot_id] = bst2;
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_, Input::K);
        Dispatch::avoidCollide = false;
        while (Input::ScanFrame()) {
            Log::print("frame", Input::frameID);
            Dispatch::UpdateCompleted();
            // Dispatch::UpdateAll();
            Dispatch::ManagePlan();
            Dispatch::ControlWalk();
            Output::Print(Input::frameID);
            assert(Input::frameID < 2000);
        }
    }
}


/*
// 测试Simulator
namespace Solution7 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace WayFinding;
    using namespace Simulator;

    void Solve() {
        Input::ScanMap();
        
        while(Input::ScanFrame()) {
            bool P = frameID < 210;
            if (P) Log::print("frame", Input::frameID);
            double forward = 6, rotate = 0;
            if (Input::frameID == 1) {
                auto robot_bak = *robot[0];
                std::function<std::pair<double,double>()> action = [=](){return std::make_pair(forward, rotate);};
                SimuFrames(robot_bak, action, 77, 1);
                Log::print(robot_bak.pos_);
            }
            if (P) Log::print(robot[0]->pos_, Length(robot[0]->linear_velocity_), robot[0]->angular_velocity_);
            Output::Forward(0, forward);
            Output::Rotate(0, rotate);
            Output::Print(Input::frameID);
        }
    }
}
*/

// 测试wayfindding
namespace Solution6 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace WayFinding;

    void Solve() {
        Input::ScanMap();
        Log::print("clock",clock());
        
        int aim_wb_gid; double dist;
        Way[0].nxt_point_wb({10,10}, 13, dist, aim_wb_gid);
        // Way[0].workbench_to_gid[13][0];
        Point aim;
        int robot_id = 0;
        Log::print("clock",clock());
        while(Input::ScanFrame()) {
            Log::enable = Input::frameID < 3000 || true;
            Log::print("frame", Input::frameID);
            auto rbt = robot[robot_id];
            if (Input::frameID == 1 || Length(aim - rbt->pos_) < 2e-1) {
                auto& w = Way[rbt->carry_id_ != 0];
                double dist;
                aim = w.nxt_point(rbt->pos_, aim_wb_gid, dist);
            }
            Log::print("aim", aim);

            double forward, rotate;
            rbt->ToPoint_1(aim, forward, rotate);

            Output::Forward(robot_id, forward);
            Output::Rotate(robot_id, rotate);

            Output::Print(Input::frameID);
        }
    }
}

namespace Solution3 {
    using namespace Input;
    using namespace Geometry;

    constexpr int profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};
    constexpr double wait_ratio_ = 20;
    constexpr int sell_limit_frame_ = 8960;
    int award_buy(int robot_id, int workbench_id) {
        return 0;
    }
    int award_sell(int robot_id, int workbench_id, int materials) {
        return 0;
    }
    int workbench_remain_num(int workbench_id) {
        int mat_id = Input::workbench[workbench_id]->type_id_;
        if (mat_id <= 3) return 0;
        int num = __builtin_popcount(Input::workbench[workbench_id]->materials_status_) +
                __builtin_popcount(Dispatch::occupy_[workbench_id].sell_occupy);
        if (mat_id <= 6) return 2 - num;
        return 3 - num;
    }
    double CalcTime(int o, Point from, int to_workbench) {
        // return Length(from - workbench[to_workbench]->pos_) / 4;
        auto& w = WayFinding::Way[o];
        double dist;
        int wbpid;
        w.nxt_point_wb(from, to_workbench, dist, wbpid);
        // Log::print("CalcTime", o, from, to_workbench, dist);
        return dist / 4;
    }
    Dispatch::Plan RobotReplan(int robot_id) {
        Log::print("RobotReplan", robot_id, Input::frameID);
        auto rb = robot[robot_id];

        Dispatch::Plan bst; bst.buy_workbench = bst.sell_workbench = -1;
        double bst_award_pf = 0; // per frame
        for (int buy_wb_id = 0; buy_wb_id < K; buy_wb_id++) {
            auto buy_wb = workbench[buy_wb_id];
            if (!buy_wb->product_status_ && buy_wb-> frame_remain_ == -1) continue; // 暂不考虑后后运送上的
            if (Dispatch::occupy_[buy_wb_id].buy_occupy) continue;
            int mat_id = buy_wb->type_id_; // 购买与出售物品id
            if (rb->carry_id_ != 0 && rb->carry_id_ != mat_id) continue;
            // 仅在搭上了顺风车才买
            // bool P = Input::frameID == 2011;
            // if (P) Log::print(mat_id, Geometry::Dist(rb->x0_, rb->y0_, buy_wb->x0_, buy_wb->y0_));
            // if (mat_id >= 4 && Geometry::Dist(rb->x0_, rb->y0_, buy_wb->x0_, buy_wb->y0_) > 1)
            //     continue;

            double buy_time = 0;
            double actual_time = 0;
            if (rb->carry_id_ == 0) {
                // actual_time = buy_time = rb->CalcTime(buy_wb->pos_);
                actual_time = buy_time = CalcTime(0, rb->pos_, buy_wb_id);
                if (!buy_wb->product_status_) // 有产品了也可以在生产时间中，故必须要此判断
                    buy_time += std::max(0.0, buy_wb-> frame_remain_ / 50.0 - buy_time) * wait_ratio_; // 等待生产
            }
            for (int sell_wb_id = 0; sell_wb_id < K; sell_wb_id++) {
                auto sell_wb = workbench[sell_wb_id];
                if (!sell_wb->TryToSell(mat_id)) continue; // 暂时只考虑能直接卖的，不考虑产品被拿走可以重新生产的
                double sell_time = 0;
                if (rb->carry_id_ == 0) {
                    // if (!sell_wb->product_status_ && sell_wb-> > 0) 
                    // TODO: 在生产，且填上当前物品就满了，则结束时间为max{到达，生产完成}。这样来到达送且拿。
                    // sell_time = rb->CalcTime(buy_wb->pos_, sell_wb->pos_);
                    sell_time = CalcTime(1, buy_wb->pos_, sell_wb_id);
                    if (false && workbench_remain_num(sell_wb_id) == 1 && !sell_wb->product_status_) {
                        // if (sell_wb->frame_remain_ / 50.0 - sell_time > 0) continue;
                        sell_time += std::max(0.0, sell_wb->frame_remain_ / 50.0 - sell_time) * wait_ratio_;
                        // 还要保证到了sellwb，能有地方需求该物品。即对占用的预测，不光有占用，还有清除的预测。
                    }
                    // sell_time -= rb->CalcTime(buy_wb->pos_);
                    actual_time += sell_time;
                    if (Input::frameID + actual_time * 50 > sell_limit_frame_) continue;
                } else {
                    sell_time = CalcTime(1, rb->pos_, sell_wb_id);
                    // sell_time = rb->CalcTime(sell_wb->pos_);
                }

                if (Dispatch::occupy_[sell_wb_id].sell_occupy >> mat_id & 1) continue;

                int award = award_buy(robot_id, buy_wb_id) +
                            award_sell(robot_id, sell_wb_id, mat_id) +
                            profit_[mat_id];

                double award_pf = award / (buy_time + sell_time);
                if (award_pf > bst_award_pf) {
                    bst_award_pf = award_pf;
                    bst.buy_workbench = buy_wb_id;
                    bst.sell_workbench = sell_wb_id;
                    bst.mat_id = mat_id;
                }
            }

        }
        if (bst.buy_workbench == bst.sell_workbench && bst.sell_workbench == -1) {
           Log::print("NoPlan", robot_id);
        } else {
           Log::print("UpdatePlan", robot_id, workbench[bst.buy_workbench]->type_id_, workbench[bst.sell_workbench]->type_id_);
        }
        return bst;
        // Dispatch::UpdatePlan(robot_id, bst);
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_, Input::K);
        Dispatch::avoidCollide = false;
        Dispatch::enableTwoPlan = false;
        // occupy.resize(K);
        // ScanFrame才初始化
        // for (size_t ri = 0; ri < Input::robot_num_; ri++) {
        //     RobotReplan(ri); // 开始规划
        // }
        while (Input::ScanFrame()) {
           Log::print("frame", Input::frameID);
            Dispatch::UpdateCompleted();
            // Dispatch::UpdateAll();
            Dispatch::ManagePlan();
            Dispatch::ControlWalk();
            Output::Print(Input::frameID);
        }
    }
}

#ifdef FALSE

namespace Solution1 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace Dispatch;

    static constexpr int robot_num_ = 4;
    static constexpr int total_frame = 15000;
    static constexpr int can_not_buy_in_last_frame = 0;
    static constexpr double inf = 1e9;
    static constexpr int frame_to_wait_in_buy = 3;
    double premium_coefficient[3] = {1.0, 1.5, 3.0};
    double sever_one;
    double four_five_six_one;
    double sever_two;
    double four_five_six_two;
    double sever_three;

    // double time_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
        return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    int can_plan_to_buy_[110], can_plan_to_buy_tmp_[110];
    int can_plan_to_sell_[110][10], can_plan_to_sell_tmp_[110][10];
    bool should_not_plan_to_buy_[110];
    /*
     * 如果A去这个商店卖，同时B规划的是去同一个商店买；可以让A卖完直接去做B的规划，让B去规划别的买的路径。
     */
    double dis_[110][110];

    // 周围 5m 之内的点
    std::vector<int> around_points[110];
    const double Around_Distance = 10.0;

    int fac[5];

    void Init() {

        // 防碰撞初始化
        switch(map_number_) {
            default:
                Dispatch::avoidCollide = false;
                Dispatch::init(nullptr, robot_num_, K);
                break;
        }

        // 第一帧开始初始化的
        if(frameID == 1) {
            for(int id = 0; id < 4; ++id) {
                robot[id] -> workbench_buy_ = -1;
                robot[id] -> workbench_sell_ = -1;
            }
            for(int i = 0; i < K; ++i) {
                can_plan_to_buy_[i] = true;
                for(int j = 1; j <= 7; ++j) {
                    can_plan_to_sell_[i][j] = true;
                }
            }
        }

        // 每帧都需要初始化的
        for(int i = 0; i < K; ++i) {
            should_not_plan_to_buy_[i] = false;
        }

        // WayFinding::Init_Frame();

        if(frameID == 1) {
            for (int i = 0; i < K; ++i) {
                for (int j = 0; j < K; ++j) {
                    if (dis_[i + robot_num_][j + robot_num_] < Around_Distance) {
                        around_points[i].emplace_back(j);
                        around_points[j].emplace_back(i);
                    }
                }
            }
        }

        fac[0] = 1; for(int i = 1; i <= 4; ++i) fac[i] = fac[i-1] * i;
    }

    void Choose_To_Point(int id, double dx, double dy, double& forward, double& rotate) {
        switch(Input::map_number_) {
            default:
                robot[id]->ToPoint_1(Geometry::Point{dx, dy}, forward, rotate);
                break;
        }
    }

    // 针对地图来忽略一些点，传入一些工作台的点坐标
    bool MissingPoint(int robot_id, int id, double x, double y) {
        switch(map_number_) {
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;

        }
        return false;
    }

    void Config_Read_From_Files() {
        //double sever_one_, four_five_six_one_, sever_two_, four_five_six_two_, sever_three_;

        FILE *fp = fopen("config.txt", "r+");
        if(fp == NULL) {
//            Log::print("Fail to open file!");
            exit(0);
        }
        fscanf(fp, "%lf%lf%lf%lf%lf%lf%lf", &sever_one, &four_five_six_one, &sever_two, &four_five_six_two, &sever_three, &premium_coefficient[1], &premium_coefficient[2]);
        fclose(fp);

//        sever_one = sever_one_;
//        four_five_six_one = four_five_six_one_;
//        sever_two = sever_two_;
//        four_five_six_two = four_five_six_two_;
//        sever_three = sever_three_;
//        Log::print(" sever_one: ", sever_one, " four_five_six_one: ", four_five_six_one, " sever_two ", sever_two,
//                   " four_five_six_two: ", four_five_six_two, " sever_three: ", sever_three);
    }

    void Config_Read_From_Files2() {
        //double sever_one_, four_five_six_one_, sever_two_, four_five_six_two_, sever_three_;

        FILE *fp = fopen("config2.txt", "r+");
        if(fp == NULL) {
//            Log::print("Fail to open file!");
            exit(0);
        }
        fscanf(fp, "%lf%lf", &premium_coefficient[1], &premium_coefficient[2]);
        fclose(fp);

//        sever_one = sever_one_;
//        four_five_six_one = four_five_six_one_;
//        sever_two = sever_two_;
//        four_five_six_two = four_five_six_two_;
//        sever_three = sever_three_;
//        Log::print(" sever_one: ", sever_one, " four_five_six_one: ", four_five_six_one, " sever_two ", sever_two,
//                   " four_five_six_two: ", four_five_six_two, " sever_three: ", sever_three);
    }

    bool AroundPoint(int idx) {
        for(const auto& idy: around_points[idx]) {
            if(workbench[idx] -> TryToSell(workbench[idy] -> type_id_) && workbench[idy] -> TryToBuy(workbench[idy] -> type_id_, -100)) {
                return true;
            }
        }
        return false;
    }

    bool Whether_Can_Buy(int id, int k, int i, int j) {
        if(can_plan_to_buy_[i] && workbench[i]->TryToBuy(k, -100) && !should_not_plan_to_buy_[i]) {
            if(can_plan_to_sell_[j][k] && workbench[j]->TryToSell(k) ) {
                // if (map_number_ == 1 && workbench[j]->type_id_ == 9) return false;
                if (MissingPoint(id, i, workbench[i]->pos_.x, workbench[i]->pos_.y) ||
                    MissingPoint(id, j, workbench[j]->pos_.x, workbench[j]->pos_.y))
                    return false; // 针对地图忽略一些点
                if (should_not_plan_to_buy_[i]) return false; // 优化：别人去卖的，你不能去买
                double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                        Geometry::Point{workbench[i]->pos_.x, workbench[i]->pos_.y},
                        Geometry::Point{workbench[j]->pos_.x, workbench[j]->pos_.y});
                if (frameID + buy_sell_frame_ > total_frame) return false;  //  没时间去卖了，所以不买。
                // double frame_to_buy_ = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i]->pos_.x, workbench[i]->pos_.y}});

                // if (frameID + buy_sell_frame_ + 300 < total_frame) { // 如果还有时间
                // if (workbench[i] -> type_id_ >= 4 && workbench[i] -> type_id_ <= 6 && AroundPoint(i)) continue; // 如果周围1,2,3的点没买，则先买。
                // }

                // if(workbench[j] -> frame_remain_ > 0 && buy_sell_frame_ < workbench[j] -> frame_remain_ && (workbench[j] -> ItemsAreMissing() == 1)) continue ;
                // Log::print("buy_sell_frame: ", buy_sell_frame_, " j: ", j, " frame_remain: ", workbench[j] -> frame_remain_, " ItemsAreMissing: ", workbench[j] -> ItemsAreMissing());
            } else return false;
        } else return false;
        return true;
    }

    bool FindItemsAreMissingLess(int id, int k, int i, int j) {
        if(workbench[j] -> type_id_ == 7) {
            for (const auto &j_: around_points[j]) {
                if (Whether_Can_Buy(id, k, i, j_) &&
                    workbench[j_]->ItemsAreMissing() < workbench[j]->ItemsAreMissing())
                    return true;
            }
        }
        return false;
    }

    void SetConfig() {
        switch (map_number_) {
            case 1:
                sever_one = 2.0;
                four_five_six_one = 1.5;
                sever_two = 1.2;
                four_five_six_two = 1.2;
                sever_three = 1.0;
                break;
            default:
                sever_one = 1.8;
                four_five_six_one = 1.6;
                sever_two = 1.3;
                four_five_six_two = 1.2;
                sever_three = 1.0;
                break;
        }
    }

    void Solve() {
        Input::ScanMap();
        SetConfig(); // 针对地图设置参数
//        Config_Read_From_Files(); // 搜参数专用
//        Config_Read_From_Files2();
        while(Input::ScanFrame()) {
            Init();
            for(int id = 0; id < 4; ++id) { // 枚举机器人
                if (robot[id] -> carry_id_) {
                    if(robot[id] -> workbench_sell_ != -1) { // 找到有工作台
                        should_not_plan_to_buy_[robot[id] -> workbench_sell_] = true;
                        // 身边有 workbench
                        if (robot[id]->workbench_ == robot[id] -> workbench_sell_) {
                            Output::Sell(id);
                            can_plan_to_sell_[robot[id] -> workbench_sell_][robot[id] -> carry_id_] = true;
                            workbench[robot[id] -> workbench_sell_] -> materials_status_ |= 1 << robot[id] -> carry_id_;
                            robot[id] -> last_point_ = WayFinding::workbench_extern_id[robot[id] -> workbench_sell_][robot[id] -> workbench_sell_direction] + robot_num_;
                            // Log::print("id: ", id, "last_point_: ", robot[id] -> last_point_);
                            robot[id] -> workbench_buy_ = robot[id] -> workbench_sell_ = -1;
                            plan_[id].sell_workbench = -1;
                            continue;
                        }
                        double forward, rotate;
                        robot[id] -> Robot_Control(forward, rotate);
                        movement_[id] = {forward, rotate};
                        plan_[id].sell_workbench = robot[id]->workbench_sell_;
                        robot[id]->workbench_buy_ = -1;
                    }
                }
            }

            /// buy
            if(frameID <= total_frame - can_not_buy_in_last_frame) {

                bool choose_to_enum_robot_order = false;
                // 枚举机器人的顺序.......
                std::vector<int> robot_need_to_plan_to_buy_;
                for (int id = 0; id < 4; ++id) { // 把没有计划的机器人记录起来，进行全排列。
                    if (robot[id]->carry_id_ == 0 && robot[id]->workbench_buy_ == -1) {
                        robot_need_to_plan_to_buy_.emplace_back(id);
                    }
                }
                std::vector<int> ans_robot_need_to_plan_to_buy_ = robot_need_to_plan_to_buy_;

                // 给工作台设置加急等级

                int premium_processing[10];
                for(int i = 0; i <= 9; ++i) premium_processing[i] = 0;
                for (int i = 0; i < K; ++i) {
                    if (workbench[i]->type_id_ == 7 && workbench[i]->ItemsAreMissing() == 1) {
                        if (!(workbench[i]->materials_status_ & (1 << 4)))
                            premium_processing[4] = std::max(2, premium_processing[4]);
                        if (!(workbench[i]->materials_status_ & (1 << 5)))
                            premium_processing[5] = std::max(2, premium_processing[5]);
                        if (!(workbench[i]->materials_status_ & (1 << 6)))
                            premium_processing[6] = std::max(2, premium_processing[6]);
                    }
                    if (workbench[i]->type_id_ == 7 && workbench[i]->ItemsAreMissing() == 2) {
                        if (!(workbench[i]->materials_status_ & (1 << 4)))
                            premium_processing[4] = std::max(1, premium_processing[4]);
                        if (!(workbench[i]->materials_status_ & (1 << 5)))
                            premium_processing[5] = std::max(1, premium_processing[5]);
                        if (!(workbench[i]->materials_status_ & (1 << 6)))
                            premium_processing[6] = std::max(1, premium_processing[6]);
                    }
                }

                int workbench_buy_direction = 0;
                int workbench_sell_direction = 0;
                for(int id : ans_robot_need_to_plan_to_buy_) {
                    // 根据策略，选一个东西去买
                    double mn = 0.0; // 物品获利 / 距离
                    int carry_id = -1, workbench_buy, workbench_sell;
                    for (int k = 1; k <= 7; ++k) { // 枚举要买的物品
                        for (int i = 0; i < K; ++i)
                            if(can_plan_to_buy_[i] && workbench[i]->TryToBuy(k, -100) && !should_not_plan_to_buy_[i]) { // 从哪个工作站买 优化：别人去卖的，你不能去买
                                for (int j = 0; j < K; ++j)
                                    if (can_plan_to_sell_[j][k] && workbench[j]->TryToSell(k)) { // 从哪个工作站卖
                                        //                                    Log::print("map_number: ", map_number_, " i: ", i , "x: ", workbench[i] -> pos_.x, "y: ", workbench[i] -> pos_.y, " MissingPoint: ", MissingPoint(i, workbench[i] -> pos_.x, workbench[i] -> pos_.y));
                                        //                                    Log::print("map_number: ", map_number_, " j: ", j , "x: ", workbench[j] -> pos_.x, "y: ", workbench[j] -> pos_.y, " MissingPoint: ", MissingPoint(j, workbench[j] -> pos_.x, workbench[j] -> pos_.y));

                                        if (!Whether_Can_Buy(id, k, i, j)) continue;

                                        double money_per_distance;
                                        /*
                                        double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                                                Geometry::Point{workbench[i]->pos_.x, workbench[i]->pos_.y},
                                                Geometry::Point{workbench[j]->pos_.x, workbench[j]->pos_.y});
                                        */
                                        auto [i_direction, j_direction] = WayFinding::dis_mn_[robot[id] -> last_point_][i][j];
                                        Log::print("i: ", i, "j: ", j, "i_direction: ", i_direction, "j_direction: ", j_direction);
                                        double buy_sell_frame_ = WayFinding::CalcDistance(id, i,
                                                                                          i_direction,
                                                                                          j,
                                                                                          j_direction);
                                        if (buy_sell_frame_ >= WayFinding::INF) continue;

                                        if ((workbench[j]->type_id_ == 7 &&
                                             workbench[j]->ItemsAreMissing() ==
                                             1)) { // 如果 7 只差一点，给一个更大的值
                                            money_per_distance =
                                                    profit_[k] * sever_one / buy_sell_frame_;
                                        } else if ((workbench[j]->type_id_ >= 4 &&
                                                    workbench[j]->type_id_ <= 6) &&
                                                   workbench[j]->ItemsAreMissing() == 1) {
                                            money_per_distance =
                                                    profit_[k] * four_five_six_one / buy_sell_frame_;
                                        } else if (workbench[j]->type_id_ == 7 &&
                                                   workbench[j]->ItemsAreMissing() == 2) {
                                            money_per_distance =
                                                    profit_[k] * sever_two / buy_sell_frame_;
                                        } else if ((workbench[j]->type_id_ >= 4 &&
                                                    workbench[j]->type_id_ <= 6) &&
                                                   workbench[j]->ItemsAreMissing() == 2) {
                                            money_per_distance =
                                                    profit_[k] * four_five_six_two / buy_sell_frame_;
                                        } else {
                                            money_per_distance = profit_[k] * 1.0 / buy_sell_frame_;
                                        }

                                        // if(map_number_ == 1 && FindItemsAreMissingLess(id, k, i, j)) continue; // 第一张图的时候，找缺更少的物品买。

                                        money_per_distance *= premium_coefficient[premium_processing[workbench[j]->type_id_]];

                                        if (money_per_distance > mn) {
                                            mn = money_per_distance;
                                            carry_id = k;
                                            workbench_buy = i;
                                            workbench_sell = j;
                                            workbench_buy_direction = i_direction;
                                            workbench_sell_direction = j_direction;
                                        }
                                    }
                            }
                    }
                    if (fabs(mn) > 1e-5) {
                        robot[id]->workbench_buy_ = workbench_buy;
                        robot[id]->workbench_sell_ = workbench_sell;
                        robot[id]->workbench_buy_direction = workbench_buy_direction;
                        robot[id]->workbench_sell_direction = workbench_sell_direction;
                        robot[id] -> v.push_back({robot[id] -> workbench_buy_, 0, workbench_buy_direction});
                        robot[id] -> v.push_back({robot[id] -> workbench_sell_, 1, workbench_sell_direction});
                        Log::print("Frame: ", frameID, "workbench_buy: ", workbench_buy, "workbench_sell: ", workbench_sell, "mn: ", mn);
                        Log::print("id: ", id, "workbench_id: ", robot[id] -> workbench_buy_, "x: ", workbench[robot[id] -> workbench_buy_] -> pos_.x, "y: ", workbench[robot[id] -> workbench_buy_] -> pos_.y);
                        Log::print("id: ", id, "workbench_id: ", robot[id] -> workbench_sell_, "x: ", workbench[robot[id] -> workbench_sell_] -> pos_.x, "y: ", workbench[robot[id] -> workbench_sell_] -> pos_.y);
                        can_plan_to_buy_[workbench_buy] = false;
                        can_plan_to_sell_[workbench_sell][carry_id] = false;
                    }
                }
                // 给机器人买计划已完成


                // 控制机器人运动到某点 或者 买东西。
                for (int id = 0; id < 4; ++id) { // 未携带物品，打算去买的。
                    if (robot[id]->carry_id_ == 0 && robot[id] -> workbench_buy_ != -1) { // 如果有则找到最优的策略，跑去买。
                        // 身边有 workbench
                        if (robot[id]->workbench_ == robot[id]->workbench_buy_) {
                            // if(workbench[robot[id] -> workbench_buy_] -> frame_remain_ > 0 && workbench[robot[id] -> workbench_buy_] -> frame_remain_ <= frame_to_wait_in_buy) continue ;
                            Output::Buy(id);
                            can_plan_to_buy_[robot[id] -> workbench_buy_] = true;
                            workbench[robot[id] -> workbench_sell_] -> product_status_ = 0;
                            robot[id] -> last_point_ = WayFinding::workbench_extern_id[robot[id] -> workbench_buy_][robot[id]->workbench_buy_direction] + robot_num_;
                            robot[id] -> workbench_buy_ = -1;
                            plan_[id].sell_workbench = -1;
                            continue;
                        }
                        double forward, rotate;
                        robot[id] -> Robot_Control(forward, rotate);
                        movement_[id] = {forward, rotate};
                        plan_[id].buy_workbench = robot[id]->workbench_buy_;
                    }
//                    else if(robot[id] -> carry_id_ == 0 && robot[id] -> workbench_buy_ == -1) { // 买不了东西，到处跑
//                        double mn = 1e9; int workbench_id = -1;
//                        for (int i = 0; i < K; ++i) if(workbench[i] -> type_id_ >= 1 && workbench[i] -> type_id_ <= 3) {
//                            if(mn > dis_[id][i + robot_num_]) {
//                                mn = dis_[id][i + robot_num_];
//                                workbench_id = i;
//                            }
//                        }
//                        if(workbench_id != -1) {
//                            double forward, rotate;
//                            Choose_To_Point(id, workbench[workbench_id]->pos_.x, workbench[workbench_id]->pos_.y, forward, rotate);
//                            movement_[id] = {forward, rotate};
//                            plan_[id].buy_workbench = workbench_id;
//                        }
//                    }
                }
            }

            // 防碰撞
            // if (avoidCollide) AvoidCollide();
            for(int id = 0; id < 4; ++id) {
                double& forward = movement_[id].first;
                double& rotate = movement_[id].second;
                // robot[id]->AvoidToWall(forward, rotate);
                Output::Forward(id, forward);
                Output::Rotate(id, rotate);
            }
            Output::Print(Input::frameID);

        }
    }
}

#endif

int main() {
    Solution4::Solve();
    return 0;
}