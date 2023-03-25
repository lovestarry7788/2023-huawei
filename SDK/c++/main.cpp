#include "log.h"
#include "geometry.h"
#include "dispatch.h"
#include "input.h"
#include "output.h"
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
#include <cassert>

// 手玩方法，呆滞数据
namespace Solution4 {
    // 走路略快，有些许错配
    std::vector<std::vector<std::pair<int,int>>> route =
    {
        {
            {16, 13},
            {16, 14},
            {16, 11},
            {16, 32},
            {16, 29},
        },
        {
            {28, 32},
            {28, 22},
            {28, 29},
            {28, 11},
            {28, 23},
            {16, 22},
            {22, 24},
        },
        {
            {16, 22},
            {16, 23},
            {16, 12},
            {16, 20},
            {16, 34},
            {28, 33},

        },
        {
            {28, 34},
            {28, 33},
            {28, 22},
            {28, 25},
            {28, 14},
            {16, 34},
        },
    };
    using namespace Input;
    void RobotReplan(int robot_id) {
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
        if (bst.buy_workbench != -1)
            Log::print("UpdatePlan", robot_id, workbench[bst.buy_workbench]->type_id_, workbench[bst.sell_workbench]->type_id_);
        Dispatch::UpdatePlan(robot_id, bst);
        Dispatch::plan2_[robot_id] = bst2;
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_, Input::K);
        Dispatch::avoidCollide = true;
        while (Input::ScanFrame()) {
            Log::print("frame", Input::frameID);
            Dispatch::UpdateCompleted();
            // Dispatch::UpdateAll();
            Dispatch::ManagePlan();
            Dispatch::ControlWalk();
            Output::Print(Input::frameID);
            int sum = 0;
            for (int i = 0 ; i < 4; i++) sum += Dispatch::plan_[i].sell_workbench != -1;
            assert(sum != 0);
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
    void RobotReplan(int robot_id) {
        // Log::print("RobotReplan", robot_id, Input::frameID);
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
                actual_time = buy_time = rb->CalcTime({Point{buy_wb->x0_, buy_wb->y0_}});
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
                    sell_time = rb->CalcTime(Point{buy_wb->x0_, buy_wb->y0_}, Point{sell_wb->x0_, sell_wb->y0_});
                    if (false && workbench_remain_num(sell_wb_id) == 1 && !sell_wb->product_status_) {
                        // if (sell_wb->frame_remain_ / 50.0 - sell_time > 0) continue;
                        sell_time += std::max(0.0, sell_wb->frame_remain_ / 50.0 - sell_time) * wait_ratio_;
                        // 还要保证到了sellwb，能有地方需求该物品。即对占用的预测，不光有占用，还有清除的预测。
                    }
                    sell_time -= rb->CalcTime(Point{buy_wb->x0_, buy_wb->y0_});
                    actual_time += sell_time;
                    if (Input::frameID + actual_time * 50 > sell_limit_frame_) continue;
                } else {
                    sell_time = rb->CalcTime(Point{sell_wb->x0_, sell_wb->y0_});
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
        Dispatch::UpdatePlan(robot_id, bst);
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_, Input::K);
        Dispatch::avoidCollide = true;
        Dispatch::enableTwoPlan = true;
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
// 测试行走
namespace Solution2 {
    using namespace Input;
    using namespace Output;
    void Solve() {
        Input::ScanMap();
        using namespace Geometry;
        std::vector<Geometry::Point> route;
        route.push_back(Geometry::Point{24.75+10,38.75});
        route.push_back(Geometry::Point{24.75,38.75});
        route.push_back(Geometry::Point{24.75+10,38.75});
        route.push_back(Geometry::Point{24.75,38.75});
        // route.push_back(Geometry::Point{24.75+10,38.75-10});
        // route.push_back(Geometry::Point{24.75+10,38.75});
        // route.push_back(Geometry::Point{24.75+10,38.75-10});
        // route.push_back(Geometry::Point{24.75+10,38.75});
        // route.push_back(Geometry::Point{24.75,38.75});
        // route.push_back(Geometry::Point{24.75+10+8.6602540378,38.75+5});
        while(Input::ScanFrame()) {
            bool P = Input::frameID < 500;
            auto robot = Input::robot[0];
            Geometry::Point loc{robot->x0_, robot->y0_};
            while (route.size() && Geometry::Length(loc - route.front()) < 3e-1) {
                route.erase(begin(route));
                Log::print("arrive");
            }
            double f, r;
            if (P) Log::print("frame", Input::frameID);
            if (route.size() >= 2) {
                robot->ToPointTwoPoint(route[0], route[1], f, r);
            } else if (route.size() >= 1)
                robot->ToPoint(route[0].x, route[0].y, f, r);
            Output::Forward(0, f);
            Output::Rotate(0, r);
            Log::print(robot->GetLinearVelocity(), robot->angular_velocity_, robot->x0_, robot->y0_);
            if (route.size()) Log::print("K", Geometry::Dist(robot->x0_, robot->y0_, route[0].x, route[0].y));
            Log::print("F", f, r);
            Output::Print(Input::frameID);
        }
        /*
        std::vector<int> estimate;
        std::vector<int> true_estimate;
        std::vector<std::vector<double>> mov;

        while(Input::ScanFrame()) {
            if (Input::frameID < 250)
                Log::print("frame", Input::frameID);
            // Solution
            Geometry::Point loc{robot[0]->x0_, robot[0]->y0_};
            while (route.size() && Geometry::Length(loc - route.front()) < 1e-1) {
                // Log::print("arrive", route.front().x, route.front().y);
                // arrive.push_back(Input::frameID);
                while (true_estimate.size() < estimate.size())
                    true_estimate.push_back(Input::frameID - true_estimate.size());
                route.erase(begin(route));
            }
            if (route.size()) {
                double forward = 0, rotate = 0;
                robot[0]->ToPoint(route.front().x, route.front().y, forward, rotate);
                estimate.push_back(round(50 * robot[0]->CalcTime(route.front())));
                Output::Forward(0, forward);
                Output::Rotate(0, rotate);
                mov.push_back({forward, rotate, robot[0]->angular_velocity_});
                // Log::print("From", robot[0]->x0_, robot[0]->y0_);
                // Log::print("Now", robot[0]->GetLinearVelocity(), robot[0]->angular_velocity_);
                // Log::print("Aim", forward, rotate);
            }
            // Log::print(Input::frameID);
            // for (auto i : Output::Operation)
            //     Log::print(i);
            // Log::print(robot[0]->orient_);

            Output::Print(Input::frameID);
        }
        for (int i = 0; i < estimate.size(); i++) {
            Log::print(i, estimate[i] - true_estimate[i],true_estimate[i]);
            // Log::print(mov[i][0], mov[i][1], mov[i][2]);
        }*/
    }
}


namespace Solution1 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace Dispatch;

    static constexpr int robot_num_ = 4;
    static constexpr int total_frame = 9000;
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
                Dispatch::avoidCollide = true;
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
        for(int idx = 0; idx < robot_num_ + K; ++idx) {
            for(int idy = 0; idy < robot_num_ + K; ++idy) {
                double sx, sy, dx, dy;
                if(idx < 4) {
                    sx = robot[idx] -> x0_;
                    sy = robot[idx] -> y0_;
                } else {
                    sx = workbench[idx - robot_num_] -> x0_;
                    sy = workbench[idx - robot_num_] -> y0_;
                }
                if(idy < 4) {
                    dx = robot[idy] -> x0_;
                    dy = robot[idy] -> y0_;
                } else {
                    dx = workbench[idy - robot_num_] -> x0_;
                    dy = workbench[idy - robot_num_] -> y0_;
                }
                dis_[idx][idy] = Distance(sx, sy, dx, dy);
                // Log::print("idx : ",idx, "idy : ", idy, " dis_[idx][idy] : ", dis_[idx][idy]);
            }
        }

        if(frameID == 1) {
            for(int i = 0; i < K; ++i) {
                for(int j = 0; j < K; ++j) {
                    if(dis_[i + robot_num_][j + robot_num_] < Around_Distance) {
                        around_points[i].emplace_back(j);
                        around_points[j].emplace_back(i);
                        Log::print(" i: ", i , " -> ", " j: ", j);
                    }
                }
            }
        }

        fac[0] = 1; for(int i = 1; i <= 4; ++i) fac[i] = fac[i-1] * i;
    }

    void Choose_To_Point(int id, double dx, double dy, double& forward, double& rotate) {
        switch(Input::map_number_) {
            default:
                robot[id]->ToPoint(dx, dy, forward, rotate);
                break;
        }
    }

    // 针对地图来忽略一些点，传入一些工作台的点坐标
    bool MissingPoint(int robot_id, int id, double x, double y) {
        switch(map_number_) {
            case 1:
                break;
                // 忽略四个角
                if(workbench[id] -> type_id_ < 4 || workbench[id] -> type_id_ > 6) return false;
                if(x <= 7.5 && y <= 42.5) return true;
                if(x <= 7.5 && y >= 42.5) return true;
                if(x >= 42.5 && y <= 7.5) return true;
                if(x >= 42.5 && y >= 42.5) return true;

            case 2:
//                if(y < 22.5) return true;
//                if(robot_id == 0 || robot_id == 1) return workbench[id]->type_id_ == 4 || workbench[id]->type_id_ == 5;
//                if(robot_id == 2) return workbench[id]->type_id_ == 4 || workbench[id]->type_id_ == 6;
//                return workbench[id]->type_id_ == 6;
                break;
                // if(x <= 7.5 || y <= 7.5 || x >= 42.5 || y >= 42.5) return true;
            case 3:
                if(robot_id == 0 || robot_id == 1) return workbench[id]->type_id_ == 4 || workbench[id]->type_id_ == 5;
                if(robot_id == 2) return workbench[id]->type_id_ == 4 || workbench[id]->type_id_ == 6;
                return workbench[id]->type_id_ == 5 || workbench[id]->type_id_ == 6;
//                if(workbench[id]->type_id_ == 4 || workbench[id]->type_id_ == 5) return true;
                //if(y > 45) return true;
                //if(x < 20) return true;
                break;
                if(y <= 15.0 && x <= 15.0) return true;
                if(y <= 15.0 && x >= 35.0) return true;
            case 4:
                //return x < 22.5;
                break;
                if(y <= 30.0) return true;

        }
        return false;
    }

    void Config_Read_From_Files() {
        //double sever_one_, four_five_six_one_, sever_two_, four_five_six_two_, sever_three_;

        FILE *fp = fopen("config.txt", "r+");
        if(fp == NULL) {
            Log::print("Fail to open file!");
            exit(0);
        }
        fscanf(fp, "%lf%lf%lf%lf%lf", &sever_one, &four_five_six_one, &sever_two, &four_five_six_two, &sever_three);
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
            Log::print("Fail to open file!");
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
                if (map_number_ == 1 && workbench[j]->type_id_ == 9) return false;
                if (MissingPoint(id, i, workbench[i]->x0_, workbench[i]->y0_) ||
                    MissingPoint(id, j, workbench[j]->x0_, workbench[j]->y0_))
                    return false; // 针对地图忽略一些点
                if (should_not_plan_to_buy_[i]) return false; // 优化：别人去卖的，你不能去买
                double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                        Geometry::Point{workbench[i]->x0_, workbench[i]->y0_},
                        Geometry::Point{workbench[j]->x0_, workbench[j]->y0_});
                if (frameID + buy_sell_frame_ > total_frame) return false;  //  没时间去卖了，所以不买。
                // double frame_to_buy_ = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i]->x0_, workbench[i]->y0_}});

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
            case 1: // 加重生产 7 的速度
                //2.00	1.6	1.3	1	1
                sever_one = 2.0;
                four_five_six_one = 1.6;
                sever_two = 1.3;
                four_five_six_two = 1.0;
                sever_three = 1.0;
                premium_coefficient[1] = 1.5;
                premium_coefficient[2] = 3.0;
                break;
            case 2:
                //1.80	1.6	1.4	1.2	1
                sever_one = 1.8;
                four_five_six_one = 1.6;
                sever_two = 1.4;
                four_five_six_two = 1.2;
                sever_three = 1.0;
                premium_coefficient[1] = 1.5;
                premium_coefficient[2] = 3;
                break;
            case 3:
                sever_one = 0;
                four_five_six_one = 1.95;
                sever_two = 0;
                four_five_six_two = 1.6;
                sever_three = 1.0;
                premium_coefficient[1] = 1.5;
                premium_coefficient[2] = 3;
                break;
            case 4:
                //1.80	1.7	1.3	1	0.8	1.5	3
                sever_one = 1.8;
                four_five_six_one = 1.7;
                sever_two = 1.3;
                four_five_six_two = 1;
                sever_three = 0.8;
                premium_coefficient[1] = 1.5;
                premium_coefficient[2] = 3;
                break;
            default:
                sever_one = 2.0;
                four_five_six_one = 1.5;
                sever_two = 1.2;
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
                             robot[id] -> workbench_buy_ = robot[id] -> workbench_sell_ = -1;
                             plan_[id].sell_workbench = -1;
                             continue;
                         }
                         double forward, rotate;
                         /*
                         robot[id]->ToPoint_1(workbench[robot[id]->workbench_sell_]->x0_, workbench[robot[id]->workbench_sell_]->y0_,
                                            forward, rotate);
                            */
                         Choose_To_Point(id, workbench[robot[id]->workbench_sell_]->x0_, workbench[robot[id]->workbench_sell_]->y0_, forward, rotate);
                         movement_[id] = {forward, rotate};
                         plan_[id].sell_workbench = robot[id] -> workbench_sell_;
                         robot[id] -> workbench_buy_ = -1;
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
                /*
                double max_total_profit_per_total_distance_ = 0.0;
                do {
                    if(!choose_to_enum_robot_order) break;
                    memcpy(can_plan_to_buy_tmp_, can_plan_to_buy_, sizeof(can_plan_to_buy_));
                    memcpy(can_plan_to_sell_tmp_, can_plan_to_sell_, sizeof(can_plan_to_sell_));
                    double total_profit_ = 0.0, total_distance_ = 0.0;
                    for(int id : robot_need_to_plan_to_buy_) {
                        // 根据策略，选一个东西去买
                        double mn = 0.0; // 物品获利 / 距离
                        int carry_id = -1, workbench_buy, workbench_sell;
                        for (int k = 1; k <= 7; ++k) { // 枚举要买的物品
                            for (int i = 0; i < K; ++i) {
                                if(can_plan_to_buy_tmp_[i] && workbench[i]->TryToBuy(k, -100) && !should_not_plan_to_buy_[i]) { // 从哪个工作站买
                                    for (int j = 0; j < K; ++j)
                                        if(can_plan_to_sell_tmp_[j][k] && workbench[j]->TryToSell(k)) { // 从哪个工作站卖
                                            // Log::print("frame: ", frameID, "times: ", times, "k: ", k, "i: ", i, "j: ", j);
                                            if (MissingPoint(i, workbench[i] -> x0_, workbench[i] -> y0_) || MissingPoint(j, workbench[j] -> x0_, workbench[j] -> y0_)) continue ; // 针对地图忽略一些点
                                            if (should_not_plan_to_buy_[i]) continue; // 优化：别人去卖的，你不能去买
                                            double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                                                    Geometry::Point{workbench[i]->x0_, workbench[i]->y0_},
                                                                Geometry::Point{workbench[j]->x0_, workbench[j]->y0_});
                                            if (frameID + buy_sell_frame_ > total_frame) continue;  //  没时间去卖了，所以不买。
                                            // double frame_to_buy_ = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i]->x0_, workbench[i]->y0_}});
                                            double money_per_distance = profit_[k] / (dis_[id][i + robot_num_] +
                                                    dis_[i + robot_num_][j +
                                                    robot_num_]);
                                            if (money_per_distance > mn) {
                                                mn = money_per_distance;
                                                carry_id = k;
                                                workbench_buy = i;
                                                workbench_sell = j;
                                            }
                                        }
                                }
                            }
                        }
                        if (fabs(mn) > 1e-5) {
                            can_plan_to_buy_tmp_[workbench_buy] = false;
                            can_plan_to_sell_tmp_[workbench_sell][carry_id] = false;
                            total_profit_ += profit_[carry_id];
                            total_distance_ += dis_[id][workbench_buy + robot_num_] + dis_[workbench_buy + robot_num_][workbench_sell + robot_num_];
                        }
                    }

                    if(fabs(total_distance_) > 1e-5) {
                        if(total_profit_ / total_distance_ > max_total_profit_per_total_distance_) {
                            max_total_profit_per_total_distance_ = total_profit_ / total_distance_;
                            ans_robot_need_to_plan_to_buy_ = robot_need_to_plan_to_buy_;
                        }
                    }
                    // Log::print("frame: ", frameID, "times: ", times, "total_profit: ", total_profit_, "total_distance: ", total_distance_);
                }while( std::next_permutation(robot_need_to_plan_to_buy_.begin(), robot_need_to_plan_to_buy_.end()) ); // 进行全排列
                */

                // 得到机器人的顺序... 正在给机器人计划去买的 workbench。

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

                for(int id : ans_robot_need_to_plan_to_buy_) {
                    // 根据策略，选一个东西去买
                    double mn = 0.0; // 物品获利 / 距离
                    int carry_id = -1, workbench_buy, workbench_sell;
                    for (int k = 1; k <= 7; ++k) { // 枚举要买的物品
                        for (int i = 0; i < K; ++i) if(can_plan_to_buy_[i] && workbench[i]->TryToBuy(k, -100) && !should_not_plan_to_buy_[i]) { // 从哪个工作站买 优化：别人去卖的，你不能去买
                                for (int j = 0; j < K; ++j) if(can_plan_to_sell_[j][k] && workbench[j]->TryToSell(k)) { // 从哪个工作站卖
//                                    Log::print("map_number: ", map_number_, " i: ", i , "x: ", workbench[i] -> x0_, "y: ", workbench[i] -> y0_, " MissingPoint: ", MissingPoint(i, workbench[i] -> x0_, workbench[i] -> y0_));
//                                    Log::print("map_number: ", map_number_, " j: ", j , "x: ", workbench[j] -> x0_, "y: ", workbench[j] -> y0_, " MissingPoint: ", MissingPoint(j, workbench[j] -> x0_, workbench[j] -> y0_));

                                    if(!Whether_Can_Buy(id, k, i, j)) continue;

                                    double money_per_distance;

                                    double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                                            Geometry::Point{workbench[i]->x0_, workbench[i]->y0_},
                                            Geometry::Point{workbench[j]->x0_, workbench[j]->y0_});

                                    if(map_number_ < 3)
                                        buy_sell_frame_ = (dis_[id][i + robot_num_] + dis_[i + robot_num_][j + robot_num_]);

                                    if((workbench[j] -> type_id_ == 7 && workbench[j] -> ItemsAreMissing() == 1)) { // 如果 7 只差一点，给一个更大的值
                                        money_per_distance = profit_[k] * sever_one / buy_sell_frame_;
                                    } else if((workbench[j] -> type_id_ >= 4 && workbench[j] -> type_id_ <= 6) && workbench[j] -> ItemsAreMissing() == 1){
                                        money_per_distance = profit_[k] * four_five_six_one / buy_sell_frame_;
                                    } else if(workbench[j] -> type_id_ == 7 && workbench[j] -> ItemsAreMissing() == 2) {
                                        money_per_distance = profit_[k] * sever_two / buy_sell_frame_;
                                    } else if((workbench[j] -> type_id_ >= 4 && workbench[j] -> type_id_ <= 6) && workbench[j] -> ItemsAreMissing() == 2) {
                                        money_per_distance = profit_[k] * four_five_six_two / buy_sell_frame_;
                                    } else {
                                        money_per_distance = profit_[k] * 1.0 / buy_sell_frame_;
                                    }

//                                    if(map_number_ == 3){
//                                        money_per_distance = profit_[k] * 1.0 / (dis_[id][i + robot_num_] + dis_[i + robot_num_][j + robot_num_]);
//                                    }

                                    // if(map_number_ == 1 && FindItemsAreMissingLess(id, k, i, j)) continue; // 第一张图的时候，找缺更少的物品买。

                                    money_per_distance *= premium_coefficient[premium_processing[workbench[j] -> type_id_]];

                                    if(map_number_ == 4 && workbench[j]->type_id_ == 4 && workbench[workbench_sell]->type_id_ != 7) money_per_distance *= 2;
//                                    if(map_number_ == 3 && k > 3) money_per_distance = 2e9;
//                                    if(map_number_ == 3 && k < 4 && workbench[j]->type_id_ == 9) money_per_distance /= 10;

                                    if (money_per_distance > mn) {
                                        mn = money_per_distance;
                                        carry_id = k;
                                        workbench_buy = i;
                                        workbench_sell = j;
                                    }
                                }
                            }
                    }
                    if (fabs(mn) > 1e-5) {
                        robot[id]->workbench_buy_ = workbench_buy;
                        robot[id]->workbench_sell_ = workbench_sell;
                        can_plan_to_buy_[workbench_buy] = false;
                        can_plan_to_sell_[workbench_sell][carry_id] = false;
                    }
                }
                Log::print("\n");
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
                            robot[id] -> workbench_buy_ = -1;
                            plan_[id].sell_workbench = -1;
                            continue;
                        }

                        double forward, rotate;
                        Choose_To_Point(id, workbench[robot[id]->workbench_buy_]->x0_, workbench[robot[id]->workbench_buy_]->y0_, forward, rotate);
                        movement_[id] = {forward, rotate};
                        plan_[id].buy_workbench = robot[id] -> workbench_buy_;
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
//                            Choose_To_Point(id, workbench[workbench_id]->x0_, workbench[workbench_id]->y0_, forward, rotate);
//                            movement_[id] = {forward, rotate};
//                            plan_[id].buy_workbench = workbench_id;
//                        }
//                    }
                }
            }

            // 防碰撞
            if (avoidCollide) AvoidCollide();
            for(int id = 0; id < 4; ++id) {
                double& forward = movement_[id].first;
                double& rotate = movement_[id].second;
                robot[id]->AvoidToWall(forward, rotate);
                Output::Forward(id, forward);
                Output::Rotate(id, rotate);
            }
            Output::Print(Input::frameID);
         }
     }
 }

 /*
 namespace Solution4 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace Dispatch;

    std::vector< std::pair<int,int> > route[4]; // 4个机器人的走路路径
    void Init() {

    }
    void Solve() {
        Init();
        while(Input::ScanFrame) {
            for(int i = 0; i < 4; ++i) {
                for(const auto& u:route[i]) {
                    if(u.second == 0) { // 规划去买
                        if (robot[i]->carry_id_ == 0 && robot[i] -> workbench_buy_ != -1) { // 如果有则找到最优的策略，跑去买。
                            // 身边有 workbench
                            if (robot[i]->workbench_ == robot[i]->workbench_buy_) {
                                Output::Buy(i);
                                continue;
                            }

                            double forward, rotate;
                            robot[i] -> ToPoint(i, workbench[robot[i]->workbench_buy_]->x0_, workbench[robot[i]->workbench_buy_]->y0_, forward, rotate);
                            movement_[i] = {forward, rotate};
                            plan_[i].buy_workbench = robot[i] -> workbench_buy_;
                        }
                    } else if(u.second == 1) { // 规划去卖

                    }
                }
            }
        }
    }
}
  */

int main() {
    Solution1::Solve();
    return 0;
}