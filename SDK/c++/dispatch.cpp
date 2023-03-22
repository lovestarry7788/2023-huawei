#include "dispatch.h"
#include "input.h"
#include "output.h"
#include "log.h"
#include "geometry.h"
#include <memory>

using namespace Dispatch;
using namespace Geometry;
// std::vector<std::vector<Geometry::Point>> Dispatch::forecast_;
std::vector<std::pair<double, double>> Dispatch::movement_;

std::vector<Plan> Dispatch::plan_;

std::vector<Occupy> Dispatch::occupy_;

bool Dispatch::avoidCollide = false;

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
        double& forward = movement_[ri].first;
        double& rotate = movement_[ri].second;
        if (wi != -1) {
            robot->ToPoint(Input::workbench[wi]->x0_, Input::workbench[wi]->y0_, forward, rotate);
        } else {
            forward = rotate = 0;
        }
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
    if (avoidCollide) AvoidCollide();
    for (size_t ri = 0; ri < plan_.size(); ri++) {
        double& forward = movement_[ri].first;
        double& rotate = movement_[ri].second;
        Input::robot[ri] -> AvoidToWall(forward, rotate);

        Log::print(ri, forward, rotate);
        Output::Forward(ri, forward);
        Output::Rotate(ri, rotate);
    }
}

double Dispatch::ForecastCollide(const std::vector<Point>& a, const std::vector<Point>& b, double mx_dist) {
    double mx = collide_dist_;
    for (int ti = 0; ti < forecast_num_; ti += forecast_per_) {
        double dist = Length(a[ti] - b[ti]);
        mx = std::min(mx, dist + ti * collide_time_elemit_);
        // TODO: 手动设置多个距离层级，
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
void Dispatch::AvoidCollide() {
    std::vector<std::vector<Point>> forecast(Input::robot_num_);

    for (int ri = 0; ri < Input::robot_num_; ri++) {
        auto robot = Input::robot[ri];
        int wi = robot->carry_id_ == 0 ? plan_[ri].buy_workbench : plan_[ri].sell_workbench;
        if (wi == -1) {
            forecast[ri] = std::vector<Point>(forecast_num_, Point{robot->x0_, robot->y0_});
        } else {
            forecast[ri] = Input::robot[ri]->ForecastToPoint(Input::workbench[wi]->x0_, Input::workbench[wi]->y0_, forecast_num_);
            // Log::print("");
            // for (auto i : forecast[ri]) Log::print(i.x, i.y);
            // forecast[ri] = Input::robot[ri]->ForecastFixed(aim_movement[ri].first, aim_movement[ri].second, forecast_num_);
        }
    }
    for (int ri = 0; ri < Input::robot_num_; ri++) {
        // if (Input::frameID >= 244 && Input::frameID <= 244+50 && ri == 0) {
        //     auto robot = Input::robot[ri];
        //     Log::print("true_pos", robot->x0_, robot->y0_);
        // }
        std::vector<int> collide_robot;
        double bst_dist = collide_dist_;
        for (int rj = ri+1; rj < Input::robot_num_; rj++) if (rj != ri) {
            double d = ForecastCollide(forecast[ri], forecast[rj], bst_dist);
            bst_dist = std::min(bst_dist, d);
            if (d < collide_dist_) {
                collide_robot.push_back(rj);
                // Log::print("ori_collide", ri, rj);
            }
        }
        // Log::print("bst_dist", bst_dist);
        if (collide_robot.empty()) continue;
        // 对robot排序，从重到轻
        collide_robot.push_back(ri);
        std::sort(begin(collide_robot), end(collide_robot), [&](int l, int r) {
            return Input::robot[l]->carry_id_ > Input::robot[r]->carry_id_;
        });
        auto movement_best = movement_;

        std::function<bool(int)> dfs = [&](int cur) {
            if (cur == collide_robot.size()) {
                double d_min = collide_dist_;
                for (int i = 0; i < collide_robot.size(); i++) 
                    for (int j = i+1; j < collide_robot.size(); j++) {
                        double d = ForecastCollide(forecast[collide_robot[i]], forecast[collide_robot[j]], bst_dist);
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
            if (dfs(cur+1)) return true;
            std::vector<std::pair<double,double>> choose= {
                {6, 0},
                {6, PI}, {6, -PI},
                {5.5, 0},
                {5.5, PI}, {5.5, -PI},
                // {6, PI/2}, {-6, PI/2},
                // {3,PI/2}, {-3,PI/2},
                // {3,PI}, 3,{-PI},
                // {0,0.5},
                // {0,PI}, 0,{-PI},
            }; // 从影响轻到重的顺序
            // sort(begin(choose), end(choose), [&](std::pair<double,double> l, std::pair<double,double> r) {
                
            // });
            int ri = collide_robot[cur];
            auto robot = Input::robot[ri];
            for (auto& [forward, rotate] : choose) {
                // if (fabs(forward - robot->GetLinearVelocity()) < 2) continue; // 要降就降猛一点，否则到时候来不及
                forecast[ri] = robot->ForecastFixed(forward, rotate, forecast_num_);
                movement_[ri] = {forward, rotate};
                if (dfs(cur+1))
                    return true;
            }
            return false;
        };
        if (dfs(0)) {
            Log::print("Hav_solution");
        } else {
            swap(movement_, movement_best); // 不换决策，以改变最大的决策作为最终状态，往往能让最小碰撞 变大。不采用
            Log::print("No_solution");
        }
    }
}
