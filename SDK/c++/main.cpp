#include "log.h"
#include "geometry.h"
#include "input.h"
#include "output.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <map>


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
    static constexpr int can_not_buy_in_last_frame = 400;

    double dis_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
      return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    bool vis_buy[110];
    std::map<std::pair<int,int> , bool> vis_sell;

    void Solve() {
        Input::ScanMap();
        while(Input::ScanFrame()) {
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

             // 初始化可以让机器人买和卖的状态。
             for(int i = 0; i < K; ++i) vis_buy[i] = false;
             vis_sell.clear();

             for(int id = 0; id < 4; ++id) { // 枚举机器人
                 if (robot[id]->carry_id_) { // 携带物品，打算去卖的
                     // 身边有 workbench
                     if (robot[id]->workbench_ != -1) { // todo: 如果不能卖，找别的地方卖，不会等。
                         if (workbench[robot[id]->workbench_]->TryToSell(robot[id]->carry_id_)) { // 可以卖出去手上的物品
                             Output::Sell(id);
                             continue;
                         }
                     }

                     // 跑去卖手上的东西
                     int workbench_id = -1;
                     double mn = 1e9;
                     for (int i = 0; i < K; ++i) { // 找一个最近的工作台
                         if (workbench[i]->TryToSell(robot[id]->carry_id_)) {
                             if (workbench_id == -1 || mn > dis_[id][i + robot_num_]) {
                                 mn = dis_[id][i + robot_num_];
                                 workbench_id = i;
                             }
                         }
                     }

                     if (workbench_id == -1) { // 如果卖不掉
                         // Log::print("id: " ,id, " carry things ", robot[id] -> carry_id_);
                         // Output::Destroy(id);
                     } else { // 找到有工作台
                         double forward, rotate;
                         Log::print("estimate time", id, workbench[workbench_id]->x0_, workbench[workbench_id]->y0_, robot[id]->CalcTime({Geometry::Point{workbench[workbench_id]->x0_, workbench[workbench_id]->y0_}}));
                         robot[id]->ToPoint(workbench[workbench_id]->x0_, workbench[workbench_id]->y0_, forward,
                                              rotate);
                         Output::Forward(id, forward);
                         Output::Rotate(id, rotate);
                     }
                 }
             }

             if(frameID <= total_frame - can_not_buy_in_last_frame) {
                 for (int id = 0; id < 4; ++id) { // 未携带物品，打算去买的。
                     if (robot[id]->carry_id_ == 0) {
                         // 身边有 workbench
                         if (robot[id]->workbench_ != -1) {
                             int carry_id = workbench[robot[id]->workbench_]->type_id_;
                             // Log::print("Can buy, id: ", id, " workbench : ", robot[id] -> workbench_, " type_id: ", workbench[robot[id] -> workbench_] -> type_id_);
                             if (workbench[robot[id]->workbench_]->TryToBuy(carry_id, -100)) { // 看看能不能买到物品
                                 // 买之前先 check 有没有地方卖
                                 bool can_sell = false;
                                 for (int i = 0; i < K; ++i) {
                                     can_sell |= workbench[i]->TryToSell(carry_id);
                                     if (can_sell) break;
                                 }

                                 if (can_sell) { // 以后可以卖的出去，买。
                                     //  Log::print("buy!, id: ", id, " workbench : ", robot[id] -> workbench_, " can_sell : ", can_sell);
                                     Output::Buy(id);
                                     continue;
                                 }
                             }
                         }

                         // 根据策略，选一个东西去买
                         double mn = 0.0; // 物品获利 / 距离
                         int carry_id = -1, workbench_buy, workbench_sell;
                         for (int k = 1; k <= 9; ++k) { // 枚举要买的物品
                             for (int i = 0; i < K; ++i) { // 从哪个工作站买
                                 for (int j = 0; j < K; ++j) { // 从哪个工作站卖
                                     if (workbench[j]->type_id_ == 9) continue;
                                     // double time_to_buy = robot[id] -> CalcTime(std::vector{Geometry::Point{workbench[i]->x0_, workbench[i] -> y0_}});
                                     if (workbench[i]->TryToBuy(k, -100) && workbench[j]->TryToSell(k)) {
                                         double money_per_distance = profit_[k] / (dis_[id][i + robot_num_] +
                                                                                   dis_[i + robot_num_][j +
                                                                                                        robot_num_]);
                                         // Log::print("things: ", k," buy from : ", i," sell from: ", j , " money_per_distance: ", money_per_distance);
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

                         if (fabs(mn - 0) > 1e-5) { // 如果有则找到最优的策略，跑去买。
                             double forward, rotate;
                             robot[id]->ToPoint(workbench[workbench_buy]->x0_, workbench[workbench_buy]->y0_, forward,
                                                rotate);
                             // if(workbench[workbench_buy] -> type_id_ >= 4 && workbench[workbench_buy] -> type_id_ <= 7)
                             //   vis_buy[workbench_buy] = true;
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