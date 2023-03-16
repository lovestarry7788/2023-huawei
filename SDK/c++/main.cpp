#include "log.h"
#include "geometry.h"
#include "dispatch.h"
#include "input.h"
#include "output.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <map>
// #include <cassert>

namespace Solution3 {
    using namespace Input;
    using namespace Geometry;

    constexpr int profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};
    int award_buy(int robot_id, int workbench_id) {
        return 0;
    }
    int award_sell(int robot_id, int workbench_id, int materials) {
        return 0;
    }
    // 工作台占用情况
    struct Occupy {
        bool buy_occupy = 0;
        int sell_occupy = 0; // >>i&1 则i物品被占用
    };
    std::vector<Occupy> occupy;
    void RobotReplan(int robot_id) {
        Log::print("RobotReplan", robot_id, Input::frameID);
        auto rb = robot[robot_id];
        auto &plan = Dispatch::plan_[robot_id];
        if ((plan.buy_workbench == -1) != (plan.sell_workbench == -1)) return;

        if (plan.buy_workbench != -1  ) {
            int bw = plan.buy_workbench;
            int sw = plan.sell_workbench;
            int mat_id = workbench[bw]->type_id_;
            occupy[bw].buy_occupy = false;
            occupy[sw].sell_occupy &= ((unsigned)1 << 31) - 1 - (1<<mat_id);
            plan.buy_workbench = plan.sell_workbench = -1;
        }
        Dispatch::Plan bst = {-1, -1};
        double bst_award_pf = 0; // per frame
        for (int buy_wb_id = 0; buy_wb_id < K; buy_wb_id++) {
            auto buy_wb = workbench[buy_wb_id];
            if (!buy_wb->product_status_) continue; // 暂不考虑后后运送上的 
            if (occupy[buy_wb_id].buy_occupy) continue;
            int mat_id = buy_wb->type_id_; // 购买与出售物品id
            int buy_frame = rb->CalcTime({Point{buy_wb->x0_, buy_wb->y0_}});
            buy_frame += std::max(0, buy_wb-> frame_remain_ - buy_frame) * 8; // 少浪费时间

            for (int sell_wb_id = 0; sell_wb_id < K; sell_wb_id++) {
                auto sell_wb = workbench[sell_wb_id];
                int sell_frame = 
                    rb->CalcTime({Point{buy_wb->x0_, buy_wb->y0_}, Point{sell_wb->x0_, sell_wb->y0_}}) - 
                    rb->CalcTime({Point{buy_wb->x0_, buy_wb->y0_}});

                if (!sell_wb->TryToSell(mat_id)) continue; // 暂时只考虑能直接卖的，不考虑产品被拿走可以重新生产的
                if (occupy[sell_wb_id].sell_occupy >> mat_id & 1) continue;

                int award = award_buy(robot_id, buy_wb_id) + 
                            award_sell(robot_id, sell_wb_id, mat_id) + 
                            profit_[mat_id];

                double award_pf = (double)award / (buy_frame + sell_frame);
                if (award_pf > bst_award_pf) {
                    bst_award_pf = award_pf;
                    bst.buy_workbench = buy_wb_id;
                    bst.sell_workbench = sell_wb_id;
                }

            }

        }
        if (bst.sell_workbench != -1) {
            int bw = bst.buy_workbench;
            int sw = bst.sell_workbench;
            int mat_id = workbench[bw]->type_id_;
            occupy[bw].buy_occupy = true;
            // assert(~occupy[sw].sell_occupy>>mat_id & 1);
            occupy[sw].sell_occupy |= (1<<mat_id);
            Dispatch::UpdatePlan(robot_id, bst);
        }
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_);
        // occupy.resize(K);
        // ScanFrame才初始化
        // for (size_t ri = 0; ri < Input::robot_num_; ri++) {
        //     RobotReplan(ri); // 开始规划
        // }
        while (Input::ScanFrame()) {
            if (occupy.size() == 0)
                occupy.resize(K);
            Dispatch::ManagePlan();
            // for (size_t ri = 0; ri < Input::robot_num_; ri++) { // 每帧重新规划，不必须
            //     RobotReplan(ri);
            // }
            Dispatch::ControlWalk();
            Output::Print(Input::frameID);
            Log::print("frame", Input::frameID);
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
        std::queue<Geometry::Point> route;
        // Geometry::Point{10,10}, Geometry::Point{40, 10}, Geometry::Point{40, 40}, Geometry::Point{10, 40}
        route.push(Geometry::Point{20,20});
        route.push(Geometry::Point{25,25});
        route.push(Geometry::Point{40,30});
        route.push(Geometry::Point{10,40});
        while(Input::ScanFrame()) {
            // Solution
            Geometry::Point loc{robot[0]->x0_, robot[0]->y0_};
            while (Geometry::Length(loc - route.front()) < 1e-1)
                route.pop();
            if (route.size()) {
                double forward = 0, rotate = 0;
                robot[0]->ToPoint(route.front().x, route.front().y, forward, rotate);

                Output::Forward(0, forward);
                Output::Rotate(0, rotate);
            }

            // Log::print(Input::frameID);
            // for (auto i : Output::Operation)
            //     Log::print(i);
            // Log::print(robot[0]->orient_);

            Output::Print(Input::frameID);
        }
    }
}


namespace Solution1 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;

    static constexpr int robot_num_ = 4;
    static constexpr int total_frame = 9000;
    static constexpr int can_not_buy_in_last_frame = 0;
    static constexpr double inf = 1e9;

    // double time_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
      return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    int can_plan_to_buy_[110];
    int can_plan_to_sell_[110][10];
    double dis_[110][110];

    void Solve() {
        Input::ScanMap();
        while(Input::ScanFrame()) {

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
                 }
            }

            // sell
            for(int id = 0; id < 4; ++id) { // 枚举机器人
                 if (robot[id] -> carry_id_) {
                     if (robot[id] -> workbench_sell_ == -1) { // 携带物品，计划卖去哪
                         // 跑去卖手上的东西
                         int workbench_id = -1;
                         double mn = 1e9;
                         for (int i = 0; i < K; ++i) { // 找一个最近的工作台
                             if (workbench[i]->TryToSell(robot[id]->carry_id_) && can_plan_to_sell_[workbench_id][robot[id] -> carry_id_]) {
                                 // double time_ = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i] -> x0_, workbench[i] -> y0_}});
                                 if (workbench_id == -1 || mn > dis_[id][i + robot_num_]) {
                                     mn = dis_[id][i + robot_num_];
                                     workbench_id = i;
                                 }
                             }
                         }
                         robot[id]->workbench_sell_ = workbench_id;
                         can_plan_to_sell_[workbench_id][robot[id] -> carry_id_] = false;
                     }

                     if (robot[id] -> workbench_sell_ == -1) { // 如果卖不掉
                         Log::print("id: ", id, " carry things ", robot[id]->carry_id_);
                         Output::Destroy(id);
                         robot[id] -> workbench_buy_ = robot[id] -> workbench_sell_ = -1;
                     } else if(robot[id] -> workbench_sell_ != -1) { // 找到有工作台
                         // 身边有 workbench
                         if (robot[id]->workbench_ == robot[id] -> workbench_sell_) {
                             Output::Sell(id);
                             can_plan_to_sell_[robot[id] -> workbench_sell_][robot[id] -> carry_id_] = true;
                             robot[id] -> workbench_buy_ = robot[id] -> workbench_sell_ = -1;
                             continue;
                         }
                         double forward, rotate;
                         robot[id]->ToPoint(workbench[robot[id]->workbench_sell_]->x0_, workbench[robot[id]->workbench_sell_]->y0_,
                                            forward, rotate);
                         robot[id]->AvoidToWall(forward, rotate);

                         Output::Forward(id, forward);
                         Output::Rotate(id, rotate);
                         robot[id] -> workbench_buy_ = -1;
                     }
                }
            }

            /// buy
            if(frameID <= total_frame - can_not_buy_in_last_frame) {
                for (int id = 0; id < 4; ++id) { // 未携带物品，打算去买的。
                    if (robot[id]->carry_id_ == 0) { // 并且它没有要买的东西
                        if (robot[id]->workbench_buy_ == -1) {
                            // 根据策略，选一个东西去买
                            double mn = 0.0; // 物品获利 / 距离
                            int carry_id = -1, workbench_buy, workbench_sell;
                            for (int k = 1; k <= 7; ++k) { // 枚举要买的物品
                                for (int i = 0; i < K; ++i) if(can_plan_to_buy_[i]) { // 从哪个工作站买，多少帧内不能去同一个地方买
                                        for (int j = 0; j < K; ++j) if(can_plan_to_sell_[j][k]) { // 从哪个工作站卖
                                            double buy_sell_frame_ = robot[id]->CalcTime(
                                                    std::vector{Geometry::Point{workbench[i]->x0_, workbench[i]->y0_},
                                                                Geometry::Point{workbench[j]->x0_,
                                                                                workbench[j]->y0_}});
                                            if (frameID + buy_sell_frame_ > total_frame) continue;  //  没时间去卖了，所以不买。
                                            if (workbench[i]->TryToBuy(k, -100) && workbench[j]->TryToSell(k)) {
                                                double money_per_distance = profit_[k] / (dis_[id][i + robot_num_] +
                                                                                          dis_[i + robot_num_][j +
                                                                                                               robot_num_]);
                                                if (money_per_distance > mn) {
                                                    mn = money_per_distance;
                                                    carry_id = k;
                                                    workbench_buy = i;
                                                    workbench_sell = j;
                                                    // goto loop1;
                                                }
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

                        if (robot[id] -> workbench_buy_ != -1) { // 如果有则找到最优的策略，跑去买。
                            // 身边有 workbench
                            if (robot[id]->workbench_ == robot[id]->workbench_buy_) {
                                Output::Buy(id);
                                can_plan_to_buy_[robot[id] -> workbench_buy_] = true;
                                robot[id] -> workbench_buy_ = -1;
                                continue;
                            }

                            double forward, rotate;
                            robot[id]->ToPoint(workbench[robot[id]->workbench_buy_]->x0_, workbench[robot[id]->workbench_buy_]->y0_, forward,
                                                 rotate);
                            robot[id]->AvoidToWall(forward, rotate);

                            Output::Forward(id, forward);
                            Output::Rotate(id, rotate);
                        }
                    }
                }
            }

            Output::Print(Input::frameID);
            Log::print("frame_id: ", frameID, " OK! ");
         }
     }
 }

int main() {
    Solution1::Solve();
    return 0;
}