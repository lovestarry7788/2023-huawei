#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "log.h"
#include "geometry.h"
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

#ifdef FALSE

using namespace Dispatch;
using namespace Geometry;
// std::vector<std::vector<Geometry::Point>> Dispatch::forecast_;
std::vector<std::pair<double, double>> Dispatch::movement_;

std::vector<Plan> Dispatch::plan_, Dispatch::plan2_;

// 当前目标在wayfindding中的id，临时的目标（-1表示无）
std::vector<int> Dispatch::graph_id_, Dispatch::temporal_graph_id_; 

std::vector<Occupy> Dispatch::occupy_;

bool Dispatch::avoidCollide = false;

bool Dispatch::enableTwoPlan = false;

// std::function<void(int)> Dispatch::RobotReplan_;
void (*Dispatch::RobotReplan_)(int);

bool fakeUpdatePlan = false;

void Dispatch::init(void (*RobotReplan)(int), int robot_num, int workbench_num) {
    RobotReplan_ = RobotReplan;
    movement_.resize(Input::robot_num_);
    plan_.resize(robot_num);
    plan2_.resize(robot_num);
    graph_id_.resize(robot_num);
    temporal_graph_id_.resize(robot_num);
    occupy_.resize(workbench_num);
}

// 被robotReplan_调用。更新plan，否则默认继承上帧plan
void Dispatch::UpdatePlan(int robot_id, Plan plan) {
    if (fakeUpdatePlan) {
        plan2_[robot_id] = plan;
        Log::print("updatefake");
        return;
    }
    // occpuy plan，可只有buy or sell
    // Log::print("UpdatePlan", robot_id, plan.buy_workbench, plan.sell_workbench);
    if (plan.buy_workbench != -1)
        occupy_[plan.buy_workbench].buy_occupy = true;
    if (plan.sell_workbench != -1)
        occupy_[plan.sell_workbench].sell_occupy |= (1<<plan.mat_id);
    // ManagePlan(robot_id, plan);
    plan_[robot_id] = plan;
}

void Dispatch::UpdateFake(int robot_id) {
    if (!enableTwoPlan) return;
    int i = robot_id;
    if (plan_[i].sell_workbench == -1) return;
    fakeUpdatePlan = true;
    Robot robot_bak = *(Input::robot[i]);
    auto robot = Input::robot[i];
    robot->carry_id_ = 0;
    auto wb = Input::workbench[plan_[i].sell_workbench];
    robot->pos_ = wb->pos_;
    // 对速度与方向暂时不调整
    RobotReplan_(i);
    Input::robot[i] = std::make_shared<Robot>(robot_bak);
    fakeUpdatePlan = false;
}
void Dispatch::UpdateAll() {
    // unoccupy all not -1 plan
    for (size_t i = 0; i < plan_.size(); i++) {
        if (plan_[i].buy_workbench != -1)   
            occupy_[plan_[i].buy_workbench].buy_occupy = false;
        if (plan_[i].sell_workbench != -1)   
            occupy_[plan_[i].sell_workbench].sell_occupy &= 
                ((unsigned)1 << 31) - 1 - (1<<plan_[i].mat_id);
    }
    std::vector<int> ord(plan_.size());
    for (int i = 0; i < ord.size(); i++) ord[i] = i;
    std::sort(begin(ord), end(ord), [&](int l, int r) {return Input::robot[l]->carry_id_ > Input::robot[r]->carry_id_;});
    for (size_t i = 0; i < plan_.size(); i++) {
        RobotReplan_(ord[i]);// 手上有东西则更新sell
        UpdateFake(ord[i]);
    }
}
void Dispatch::UpdateCompleted() {
    for (size_t i = 0; i < plan_.size(); i++) {
        auto plan = plan_[i];
        if (plan.buy_workbench == -1 && plan.sell_workbench == -1 && RobotReplan_) {
            RobotReplan_(i);
            UpdateFake(i);
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

    auto robot = Input::robot[robot_id];
    int wi = robot->carry_id_ == 0 ? plan.buy_workbench : plan.sell_workbench;
    Log::print("ManagePlan", robot_id, robot->workbench_, plan.buy_workbench, plan.sell_workbench);
    if (robot->workbench_ == wi && wi != -1) { // 距离可以进行买卖
        if (robot->carry_id_ == 0 && Input::workbench[wi]->product_status_) {
            Output::Buy(robot_id);
            occupy_[wi].buy_occupy = false;
            // Plan out = plan; out.sell_workbench = -1;
            // unoccupy_queue_.push(out);
            plan.buy_workbench = -1;
        } else if (robot->carry_id_ != 0 && Input::workbench[wi]->TryToSell(robot->carry_id_)) {
            Output::Sell(robot_id);
            occupy_[wi].sell_occupy &= ((unsigned)1 << 31) - 1 - (1<<robot->carry_id_);
            // Plan out = plan; out.buy_workbench = -1;
            // unoccupy_queue_.push(out);
            plan.sell_workbench = -1;
        } 
    }
}

std::pair<double,double> Dispatch::ChooseToPointFix(int ri, int graphid) {
    double forward, rotate;
    if (graphid == -1) {
        forward = rotate = 0;
    } else {
        auto robot = Input::robot[ri];
        robot->ToPoint(WayFinding::nxt_point(robot->carry_id_ != 0, robot->pos_, graphid) , forward, rotate);
    }
    return std::make_pair(forward, rotate);
}
std::pair<double,double> Dispatch::ChooseToPoint(int ri) {
    auto robot = Input::robot[ri];
    double forward, rotate;
    // 修改成依赖graph_id而非plan
    int toid = temporal_graph_id_[ri] != -1 ? temporal_graph_id_[ri] : graph_id_[ri];
    return ChooseToPointFix(ri, toid);

    // int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
    // Log::print("ControlWalk", ri, plan_[ri].buy_workbench, plan_[ri].sell_workbench);
    // if (wi != -1) {
        // int wi2 = robot->carry_id_ == 0 ? plan_[ri].sell_workbench : plan2_[ri].buy_workbench;
        // if (wi2 != -1) {
        //     int frame_reach = 0;
        //     if (robot->carry_id_ == 0 && Input::workbench[wi]->frame_remain_ != -1 && !Input::workbench[wi]->product_status_)
        //         frame_reach = Input::frameID + Input::workbench[wi]->frame_remain_;
        //     else if (robot->carry_id_ != 0 && !Input::workbench[wi]->TryToSell(robot->carry_id_))
        //         frame_reach = INT_MAX;
        //     robot->ToPointTwoPoint(Input::workbench[wi]->pos_, Input::workbench[wi2]->pos_, forward, rotate, frame_reach);
        // }
        // else {
            // Log::print("oneToPoint");
            // robot->ToPoint(WayFinding::nxt_point(robot->carry_id_ != 0, robot->pos_, WayFinding::get_workbench_id(wi)) , forward, rotate);
        // }
        // if (robot->carry_id_ == 0) {
        //     robot->ToPointTwoPoint(Geometry::Point{Input::workbench[wi]->x0_, Input::workbench[wi]->y0_}, Geometry::Point{Input::workbench[plan_[ri].sell_workbench]->x0_, Input::workbench[plan_[ri].sell_workbench]->y0_}, forward, rotate);
        // } else {
        // }
        
    // } else {
    //     forward = rotate = 0;
    // }
    // Input::robot[ri] -> AvoidToWall(forward, rotate);
    return std::make_pair(forward, rotate);
}

// 输出行走
void Dispatch::ControlWalk() {
    
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
        graph_id_[ri] = WayFinding::get_workbench_id(wi);
    }
    if (avoidCollide) AvoidCollide();
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        movement_[ri] = ChooseToPoint(ri);
        Log::print("ControlWalk", ri, movement_[ri].first, movement_[ri].second);
    }
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        double& forward = movement_[ri].first;
        double& rotate = movement_[ri].second;
        // Input::robot[ri] -> AvoidToWall(forward, rotate);

        // Log::print(ri, forward, rotate);
        Output::Forward(ri, forward);
        Output::Rotate(ri, rotate);
    }
}


double Dispatch::ForecastCollide(const std::vector<Point>& a, const std::vector<Point>& b, double mx_dist) {
    double mx = collide_dist_;
    for (int ti = 0; ti < forecast_num_; ti += forecast_per_) {
        double dist = Length(a[ti] - b[ti]);
        mx = std::min(mx, dist + ti * collide_time_elemit_);
        if (mx < mx_dist) return mx;
        // 2 - 0 * t
        // if (dist < 1.7 - ti * 0.03) { // 越小对路线估计要求越高，越大越浪费时间 
            // if (Input::frameID == 246)
            //     Log::print(ti, dist, a[ti].x, a[ti].y, b[ti].x, b[ti].y);
            // return true;
        // }
    }
    return mx;
    // return false;
}

// std::vector<int> route_; // 值为WayFinding中的图id
// int route_cnt_; // 下一个目标点id

void Dispatch::AvoidCollide() {
    std::vector<std::vector<std::vector<Point>>> forecast(Input::robot_num_);
    // t
    for (int ri = 0; ri < Input::robot_num_; ri++) {
        std::function<std::pair<double,double>(Robot&)> action = [&](Robot&) {
            return ChooseToPointFix(ri, graph_id_[ri]); // 强制按原来的计划试试
        };
        forecast[ri].push_back(Simulator::SimuFrames(*Input::robot[ri], action, forecast_num_, forecast_sampling_));
    }
    for (int ri = 0; ri < Input::robot_num_; ri++) {
        // if (Input::frameID >= 244 && Input::frameID <= 244+50 && ri == 0) {
        //     auto robot = Input::robot[ri];
        //     Log::print("true_pos", robot->x0_, robot->y0_);
        // }
        std::vector<int> collide_robot;
        double bst_dist = collide_dist_;
        for (int rj = ri+1; rj < Input::robot_num_; rj++) if (rj != ri) {
            double d = ForecastCollide(forecast[ri][0], forecast[rj][0], bst_dist);
            bst_dist = std::min(bst_dist, d);
            if (d < collide_dist_) {
                collide_robot.push_back(rj);
                // Log::print("ori_collide", ri, rj);
            }
        }
        // Log::print("bst_dist", bst_dist);
        if (collide_robot.empty()) {
            temporal_graph_id_[ri] = -1; // 去掉临时目标
            continue;
        }
        if (temporal_graph_id_[ri] != -1) { // 已设置计划
            // continue?
        }
        // 对robot排序，从重到轻
        collide_robot.push_back(ri);
        std::sort(begin(collide_robot), end(collide_robot), [&](int l, int r) {
            return Input::robot[l]->carry_id_ > Input::robot[r]->carry_id_;
        });

        // 延长检测为安全确保的时间
        for (auto i : collide_robot) {
            std::function<std::pair<double,double>()> action = [&]() {
                return ChooseToPoint(i); // 按目前计划走
            };
            forecast[i][0] = Simulator::SimuFrames(*Input::robot[i], action, forecast_oneway_safe_num_, forecast_sampling_);
        }

        auto temporal_graph_id_best = temporal_graph_id_;
        std::function<bool(int,std::vector<int>)> dfs = [&](int cur, std::vector<int> dec) {
            if (cur == collide_robot.size()) {
                double d_min = collide_dist_;
                for (int i = 0; i < collide_robot.size(); i++) 
                    for (int j = i+1; j < collide_robot.size(); j++) {
                        double d = ForecastCollide(forecast[collide_robot[i]][dec[i]], forecast[collide_robot[j]][dec[j]], bst_dist);
                        if (d < bst_dist) return false;
                        d_min = std::min(d_min, d);
                    }
                // Log::print("not better", bst_dist, d_min);
                if (d_min > bst_dist) {
                    bst_dist = d_min;
                    movement_best = movement_;
                }
                if (d_min < collide_dist_)
                    return false;
                return true;
            }
            int ri = collide_robot[cur];
            auto robot = Input::robot[ri];
            if (dfs(cur+1, dec)) return true;
            std::vector<std::pair<double,double>> choose= {
                {6, 0},
                {6, PI}, {6, -PI},
                {5, PI/2}, {5, -PI/2},
                // {5.5, PI}, {5.5, -PI},
                // {6, PI/2}, {-6, PI/2},
                // {3,PI/2}, {-3,PI/2},
                // {3,PI}, 3,{-PI},
                // {0,0.5},
                // {0,PI}, 0,{-PI},
            }; // 从影响轻到重的顺序
            // sort(begin(choose), end(choose), [&](std::pair<double,double> l, std::pair<double,double> r) {
                
            // });
            for (int ci = 0; ci < choose.size(); ci++) {
                auto forward = choose[ci].first;
                auto rotate = choose[ci].second;
                // if (fabs(forward - robot->GetLinearVelocity()) < 2) continue; // 要降就降猛一点，否则到时候来不及
                if (forecast[ri].size() <= ci+1) {
                    std::function<std::pair<double,double>()> action = [&]() {
                        return std::make_pair(forward, rotate);
                    };
                    forecast[ri].push_back(Simulator::SimuFrames(*Input::robot[ri], action, forecast_num_, forecast_sampling_));
                }
                movement_[ri] = {forward, rotate};
                auto dec2 = dec;
                dec2[cur] = ci+1;
                if (dfs(cur+1, dec2))
                    return true;
            }
            return false;
        };
        std::function<bool(int,std::vector<int>)> dfs2 = [&](int cur, std::vector<int> dec) {
            if (cur == collide_robot.size()) {
                double d_min = collide_dist_;
                for (int i = 0; i < collide_robot.size(); i++) 
                    for (int j = i+1; j < collide_robot.size(); j++) {
                        double d = ForecastCollide(forecast[collide_robot[i]][dec[i]], forecast[collide_robot[j]][dec[j]], bst_dist);
                        if (d < bst_dist) return false;
                        d_min = std::min(d_min, d);
                    }
                // Log::print("not better", bst_dist, d_min);
                if (d_min > bst_dist) {
                    bst_dist = d_min;
                    temporal_graph_id_best = temporal_graph_id_;
                }
                if (d_min < collide_dist_)
                    return false;
                return true;
            }
            int ri = collide_robot[cur];
            auto robot = Input::robot[ri];
            // int source = route_[route_cnt_ - 1];
            if (dfs2(cur+1, dec)) return true;
            int o = robot->carry_id_ != 0;
            // 从重要到轻，每次层的上限为上层的2（可调参数）倍。暂定优先级最高的不搜路径。
            const auto& dist_order = dist_order_[o][WayFinding::get_random_id(robot->pos_)];
            int maxci = std::min(cur == 0 ? 1ull : forecast[cur-1].size() * 2, dist_order.size());
            for (int ci = 0; ci < maxci; ci++) {
                temporal_graph_id_[ri] = dist_order[ci];
                if (forecast[ri].size() <= ci+1) {
                    std::function<std::pair<double,double>(Robot&)> action = [&](Robot& robot) {
                        double forward, rotate;
                        robot->ToPoint(WayFinding::nxt_point(robot->carry_id_ != 0, robot->pos_, temporal_graph_id_[ri]) , forward, rotate);
                        return std::make_pair(forward, rotate);
                    };
                    forecast[ri].push_back(Simulator::SimuFrames(*Input::robot[ri], action, forecast_oneway_safe_num_, forecast_sampling_));
                }
                dec[ri] = ci+1;
                if (dfs2(cur+1, dec)) return true;
            }

        };
        if (dfs2(0, std::vector<int>(collide_robot.size()))) {
            Log::print("Hav_solution");
        } else {
            swap(temporal_graph_id_, temporal_graph_id_best);
            Log::print("No_solution");
        }
    }
}


// Route route_;
// void Dispatch::AvoidStuck() {

// }

#endif