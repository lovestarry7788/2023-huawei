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
            double buy_time = 0;
            if (rb->carry_id_ == 0) {
                buy_time = rb->CalcTime({Point{buy_wb->x0_, buy_wb->y0_}});
                buy_time += std::max(0.0, buy_wb-> frame_remain_ / 50.0 - buy_time) * 8; // 少浪费时间
            }

            for (int sell_wb_id = 0; sell_wb_id < K; sell_wb_id++) {
                auto sell_wb = workbench[sell_wb_id];
                double sell_time = 0;
                if (rb->carry_id_ == 0) {
                    sell_time = 
                        rb->CalcTime(Point{buy_wb->x0_, buy_wb->y0_}, Point{sell_wb->x0_, sell_wb->y0_}) - 
                        rb->CalcTime(Point{buy_wb->x0_, buy_wb->y0_});
                } else {
                    sell_time = rb->CalcTime(Point{sell_wb->x0_, sell_wb->y0_});
                }

                if (!sell_wb->TryToSell(mat_id)) continue; // 暂时只考虑能直接卖的，不考虑产品被拿走可以重新生产的
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
                // if (Input::frameID == 51 && robot_id == 2) {
                //     Log::print(award, award_pf, buy_wb_id, sell_wb_id, sell_wb->type_id_);
                //     Log::print(buy_time, sell_time);
                // }
            }

        }
        Log::print("UpdatePlan", robot_id, bst.buy_workbench, bst.sell_workbench);
        Dispatch::UpdatePlan(robot_id, bst);
    }
    void Solve() {
        Input::ScanMap();
        Dispatch::init(RobotReplan, Input::robot_num_, Input::K);
        // occupy.resize(K);
        // ScanFrame才初始化
        // for (size_t ri = 0; ri < Input::robot_num_; ri++) {
        //     RobotReplan(ri); // 开始规划
        // }
        while (Input::ScanFrame()) {
            // Dispatch::UpdateCompleted();
            Log::print("frame", Input::frameID);
            Dispatch::UpdateAll();
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
    static constexpr int frame_to_wait_in_buy = 3;

    // double time_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
      return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    int can_plan_to_buy_[110];
    int can_plan_to_sell_[110][10];
    bool should_not_plan_to_buy_[110];
    /*
     * 如果A去这个商店卖，同时B规划的是去同一个商店买；可以让A卖完直接去做B的规划，让B去规划别的买的路径。
     */
    double dis_[110][110];

    void Init() {
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
            }
        }
    }

    void Choose_To_Point(int id, double dx, double dy, double& forward, double& rotate) {
        switch(Input::map_number_) {
            /*
            case 2:
                robot[id]->ToPoint_1(dx, dy, forward, rotate);
                // Log::print("Choose_ToPoint_1\n");
                break;
            */
            default:
                robot[id]->ToPoint(dx, dy, forward, rotate);
                // Log::print("Choose_ToPoint\n");
                break;
        }
    }

    void Solve() {
        Input::ScanMap();
        while(Input::ScanFrame()) {
            Init();

            // sell
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
                             continue;
                         }
                         double forward, rotate;
                         /*
                         robot[id]->ToPoint_1(workbench[robot[id]->workbench_sell_]->x0_, workbench[robot[id]->workbench_sell_]->y0_,
                                            forward, rotate);
                            */
                         Choose_To_Point(id, workbench[robot[id]->workbench_sell_]->x0_, workbench[robot[id]->workbench_sell_]->y0_, forward, rotate);
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
                                            if (should_not_plan_to_buy_[i]) continue; // 优化：别人去卖的，你不能去买
                                            double buy_sell_frame_ = 50 * robot[id]->CalcTime(
                                                    Geometry::Point{workbench[i]->x0_, workbench[i]->y0_},
                                                                Geometry::Point{workbench[j]->x0_, workbench[j]->y0_});
                                            if (frameID + buy_sell_frame_ > total_frame) continue;  //  没时间去卖了，所以不买。
                                            // double frame_to_buy_ = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i]->x0_, workbench[i]->y0_}});
                                            if (workbench[i]->TryToBuy(k, -100) && workbench[j]->TryToSell(k)) {
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
                                robot[id]->workbench_buy_ = workbench_buy;
                                robot[id]->workbench_sell_ = workbench_sell;
                                can_plan_to_buy_[workbench_buy] = false;
                                can_plan_to_sell_[workbench_sell][carry_id] = false;
                            }
                        }
                        
                        if (robot[id] -> workbench_buy_ != -1) { // 如果有则找到最优的策略，跑去买。
                            // 身边有 workbench
                            if (robot[id]->workbench_ == robot[id]->workbench_buy_) {
                                // if(workbench[robot[id] -> workbench_buy_] -> frame_remain_ > 0 && workbench[robot[id] -> workbench_buy_] -> frame_remain_ <= frame_to_wait_in_buy) continue ;
                                Output::Buy(id);
                                can_plan_to_buy_[robot[id] -> workbench_buy_] = true;
                                workbench[robot[id] -> workbench_sell_] -> product_status_ = 0;
                                robot[id] -> workbench_buy_ = -1;
                                continue;
                            }

                            double forward, rotate;
                            /*
                            robot[id]->ToPoint_1(workbench[robot[id]->workbench_buy_]->x0_, workbench[robot[id]->workbench_buy_]->y0_, forward,
                                                 rotate);
                             */
                            Choose_To_Point(id, workbench[robot[id]->workbench_buy_]->x0_, workbench[robot[id]->workbench_buy_]->y0_, forward, rotate);
                            robot[id]->AvoidToWall(forward, rotate);

                            Output::Forward(id, forward);
                            Output::Rotate(id, rotate);
                        }
                    }
                }
            }

            Output::Print(Input::frameID);
         }
     }
 }

int main() {
    Solution2::Solve();
    return 0;
}