//
// Created by 刘智杰 on 2023/3/29.
//

#include "log.h"
#include "geometry.h"
#include "dispatch.h"
#include "input.h"
#include "output.h"
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


// 测试wayfindding
namespace Solution6 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;
    using namespace WayFinding;

    void Solve() {
        Input::ScanMap();
        int u = 0; // 表示第 0 号机器人
        std::vector<int> v = {0, 9, 11, 15, 17, 22, 25, 26, 30, 29}; // 去第 0 号工作台
        Route route;

        while(Input::ScanFrame()) {
            Log::print("frame", Input::frameID);
            if (route.empty()) {
                if (v.empty() || !GetRoute(robot[u]->pos_, v.front(), route))
                    Log::print("find route failed");
                Log::print("routes_size: ", route.size());
                for(const auto& point: route) {
                    Log::print(point.x, point.y);
                }
            }
            Point robot_pos = robot[0] -> pos_;
            while (route.size() && Geometry::Length(robot_pos - route.front()) < 0.1) { // 机器人到达某个点，则删掉。
                Log::print("reach");
                route.erase(begin(route)); // 不能直接删除，可能需要回滚
                if (route.empty()) 
                    v.erase(begin(v));
            }
            double forward, rotate;
//            if (route.size() >= 2) {
//                robot->ToPointTwoPoint(route[0], route[1], f, r);
//            } else if (route.size() >= 1)
            robot[0]->ToPoint_1(route.front(), forward, rotate);
            Output::Forward(0, forward);
            Output::Rotate(0, rotate);
            Output::Print(Input::frameID);
        }

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

        /*
        for(int idx = 0; idx < robot_num_ + K; ++idx) {
            for(int idy = 0; idy < robot_num_ + K; ++idy) {
                double sx, sy, dx, dy;
                if(idx < 4) {
                    sx = robot[idx] -> pos_.x;
                    sy = robot[idx] -> pos_.y;
                } else {
                    sx = workbench[idx - robot_num_] -> pos_.x;
                    sy = workbench[idx - robot_num_] -> pos_.y;
                }
                if(idy < 4) {
                    dx = robot[idy] -> pos_.x;
                    dy = robot[idy] -> pos_.y;
                } else {
                    dx = workbench[idy - robot_num_] -> pos_.x;
                    dy = workbench[idy - robot_num_] -> pos_.y;
                }
                dis_[idx][idy] = Distance(sx, sy, dx, dy);
            }
        }
        */
        WayFinding::Init_Frame();

        if(frameID == 1) {
            for(int i = 0; i < K; ++i) {
                for(int j = 0; j < K; ++j) {
                    if(dis_[i + robot_num_][j + robot_num_] < Around_Distance) {
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
                robot[id]->ToPoint(Geometry::Point{dx, dy}, forward, rotate);
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
                if (map_number_ == 1 && workbench[j]->type_id_ == 9) return false;
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
                        while (robot[id] -> route_.size() && Geometry::Length(robot[id] -> pos_ - robot[id] -> route_.front()) < 0.1) { // 机器人到达某个点，则删掉。
                            Log::print("1, robot_id: ", id, "plan_to_sell: ", robot[id] -> workbench_sell_, "px: ", robot[id] -> pos_.x, "py: ", robot[id] -> pos_.y, "route.x: ", robot[id] -> route_.front().x, "route.y: ", robot[id] -> route_.front().y, "dis: ",Geometry::Length(robot[id] -> pos_ - robot[id] -> route_.front()));
                            robot[id] -> route_.erase(begin(robot[id] -> route_)); // 不能直接删除，可能需要回滚
                        }
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
                        if(robot[id] -> route_.size()) {
                            Log::print("robot_id: ", id, "plan_to_sell: ", robot[id]->workbench_sell_, "px: ",
                                       robot[id]->pos_.x, "py: ", robot[id]->pos_.y, "route.x: ",
                                       robot[id]->route_.front().x, "route.y: ", robot[id]->route_.front().y, "dis: ",
                                       Geometry::Length(robot[id]->pos_ - robot[id]->route_.front()));
                            for (const auto &u: robot[id]->route_) {
                                Log::print(u.x, u.y);
                            }
                            Choose_To_Point(id, robot[id]->route_.front().x, robot[id]->route_.front().y, forward,
                                            rotate);
                            movement_[id] = {forward, rotate};
                            plan_[id].sell_workbench = robot[id]->workbench_sell_;
                            robot[id]->workbench_buy_ = -1;
                        }
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

                for(int id : ans_robot_need_to_plan_to_buy_) {
                    // 根据策略，选一个东西去买
                    double mn = 0.0; // 物品获利 / 距离
                    int carry_id = -1, workbench_buy, workbench_sell;
                    for (int k = 1; k <= 7; ++k) { // 枚举要买的物品
                        for (int i = 0; i < K; ++i) if(can_plan_to_buy_[i] && workbench[i]->TryToBuy(k, -100) && !should_not_plan_to_buy_[i]) { // 从哪个工作站买 优化：别人去卖的，你不能去买
                                for (int j = 0; j < K; ++j) if(can_plan_to_sell_[j][k] && workbench[j]->TryToSell(k)) { // 从哪个工作站卖
//                                    Log::print("map_number: ", map_number_, " i: ", i , "x: ", workbench[i] -> pos_.x, "y: ", workbench[i] -> pos_.y, " MissingPoint: ", MissingPoint(i, workbench[i] -> pos_.x, workbench[i] -> pos_.y));
//                                    Log::print("map_number: ", map_number_, " j: ", j , "x: ", workbench[j] -> pos_.x, "y: ", workbench[j] -> pos_.y, " MissingPoint: ", MissingPoint(j, workbench[j] -> pos_.x, workbench[j] -> pos_.y));

                                        if(!Whether_Can_Buy(id, k, i, j)) continue;

                                        double money_per_distance;
                                        /*
                                        double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                                                Geometry::Point{workbench[i]->pos_.x, workbench[i]->pos_.y},
                                                Geometry::Point{workbench[j]->pos_.x, workbench[j]->pos_.y});

                                        if(map_number_ < 3)
                                            buy_sell_frame_ = (dis_[id][i + robot_num_] + dis_[i + robot_num_][j + robot_num_]);
                                        */
                                        double buy_sell_frame_ = WayFinding::CalcDistance(id, i, j);
                                        // Log::print("i: ", i, "j: ", j, "dis: ", WayFinding::CalcDistance(id, i, j));
                                        if(buy_sell_frame_ >= WayFinding::INF) continue;

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

                                        // if(map_number_ == 1 && FindItemsAreMissingLess(id, k, i, j)) continue; // 第一张图的时候，找缺更少的物品买。

                                        money_per_distance *= premium_coefficient[premium_processing[workbench[j] -> type_id_]];

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
                        Log::print("robot_id: ", id, "workbench_buy: ", workbench_buy, "workbench_sell: ", workbench_sell, "dis: ", mn);
                        WayFinding::GetRoute(robot[id]->pos_, robot[id]->workbench_buy_, robot[id] -> route_);
                        can_plan_to_buy_[workbench_buy] = false;
                        can_plan_to_sell_[workbench_sell][carry_id] = false;
                    }
                }
                //Log::print("\n");
                // 给机器人买计划已完成


                // 控制机器人运动到某点 或者 买东西。
                for (int id = 0; id < 4; ++id) { // 未携带物品，打算去买的。
                    if (robot[id]->carry_id_ == 0 && robot[id] -> workbench_buy_ != -1) { // 如果有则找到最优的策略，跑去买。
                        while (robot[id] -> route_.size() && Geometry::Length(robot[id] -> pos_ - robot[id] -> route_.front()) < 0.1) { // 机器人到达某个点，则删掉。
                            robot[id] -> route_.erase(begin(robot[id] -> route_)); // 不能直接删除，可能需要回滚
                        }
                        // 身边有 workbench
                        if (robot[id]->workbench_ == robot[id]->workbench_buy_) {
                            // if(workbench[robot[id] -> workbench_buy_] -> frame_remain_ > 0 && workbench[robot[id] -> workbench_buy_] -> frame_remain_ <= frame_to_wait_in_buy) continue ;
                            Output::Buy(id);
                            can_plan_to_buy_[robot[id] -> workbench_buy_] = true;
                            workbench[robot[id] -> workbench_sell_] -> product_status_ = 0;
                            robot[id] -> workbench_buy_ = -1;
                            WayFinding::GetRoute(robot[id] -> pos_, robot[id] -> workbench_sell_, robot[id] -> route_);

                            Log::print("robot_id: ", id, "plan_to_sell: ", robot[id] -> workbench_sell_);
                            for(const auto& u: robot[id] -> route_) {
                                Log::print(u.x, u.y);
                            }

                            plan_[id].sell_workbench = -1;
                            continue;
                        }
                        double forward, rotate;
                        if(robot[id] -> route_.size()) {
                            Choose_To_Point(id, robot[id]->route_.front().x, robot[id]->route_.front().y, forward,
                                            rotate);
                            movement_[id] = {forward, rotate};
                            plan_[id].buy_workbench = robot[id]->workbench_buy_;
                        }
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

int main() {
    Solution1::Solve();
    return 0;
}