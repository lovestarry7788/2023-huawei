#include "robot.h"
#include "workbench.h"
#include "log.h"
#include "geometry.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string_view>
#include <fstream>
#include <queue>

namespace MLog {
    std::ofstream ofs("main.log");
    template<class T, class... A> void print(T&& x, A&&... a){ 
        ofs<<x; (int[]){(ofs<< ' '<< a,0)...}; ofs<<'\n'; 
    }
}

namespace Input {
    int frameID, coins, K;
    char map_[101][101];
    std::vector<std::shared_ptr<Workbench> > workbench;
    std::vector<std::shared_ptr<Robot> > robot;

    bool readUntilOK() {
        char line[1024];
        while (fgets(line, sizeof line, stdin)) {
            if (line[0] == 'O' && line[1] == 'K') {
                return true;
            }
            //do something
        }
        return false;
    }

    void ScanMap() {
        // 地图输入开始
        for(int i = 0; i < 100; ++i) {
            for(int j = 0; j < 100; ++j) {
                scanf("\n%c",&map_[i][j]);
            }
        }
        readUntilOK();
        puts("OK");
        fflush(stdout);
    }

    bool ScanFrame() { 
        while (scanf("%d%d", &frameID, &coins) != EOF) {
            scanf("%d",&K);
            workbench.resize(K);
            for(int i = 0; i < K; ++i) {
                int type_id, frame_remain, materials_status, product_status;
                double x0, y0;
                scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
                if (!workbench[i])
                    workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
                else {
                    workbench[i]->type_id_ = type_id;
                    workbench[i]->x0_ = x0;
                    workbench[i]->y0_ = y0;
                    workbench[i]->frame_remain_ = frame_remain;
                    workbench[i]->materials_status_ = materials_status;
                    workbench[i]->product_status_ = product_status;
                }
            }

            robot.resize(4);
            for(int id = 0; id < 4; ++id) {
                int workbench, carry_id;
                double time_coefficient, collide_coefficient, angular_velocity, linear_velocity_x, linear_velocity_y, orient, x0, y0;
                scanf("%d%d%lf%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity_x, &linear_velocity_y, 
                      &orient, &x0, &y0);
                if (!robot[id]) 
                    robot[id] = std::make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity_x, linear_velocity_y, orient, x0, y0);
                else {
                    robot[id]->id_ = id;
                    robot[id]->workbench_ = workbench;
                    robot[id]->carry_id_ = carry_id;
                    robot[id]->time_coefficient_ = time_coefficient;
                    robot[id]->collide_coefficient_ = collide_coefficient;
                    robot[id]->angular_velocity_ = angular_velocity;
                    robot[id]->linear_velocity_x_ = linear_velocity_x;
                    robot[id]->linear_velocity_y_ = linear_velocity_y;
                    robot[id]->orient_ = orient;
                    robot[id]->x0_ = x0;
                    robot[id]->y0_ = y0;
                }
            }

            readUntilOK();
            return true;
        }
        return false;
    }
};

namespace Output {
    std::vector<std::string> Operation; // string_view会乱码，故换成string --WY

    void Forward(int robot_id, double velocity) {
        Operation.emplace_back(std::string{"forward " + std::to_string(robot_id) + " " + std::to_string(velocity)});
    }

    void Rotate(int robot_id, double radius) {
        Operation.emplace_back(std::string{"rotate " + std::to_string(robot_id) + " " + std::to_string(radius)});
    }

    void Buy(int robot_id) {
        Operation.emplace_back(std::string{"buy " + std::to_string(robot_id)});
    }

    void Sell(int robot_id) {
        Operation.emplace_back(std::string{"sell " + std::to_string(robot_id)});
    }

    void Destroy(int robot_id) {
        Operation.emplace_back(std::string{"destory " + std::to_string(robot_id)});
    }

    void Print(int frame_id) {
        printf("%d\n", frame_id);
        for(const auto& u: Operation) {
            printf("%s\n", u.data()); // 记得换行
        }
        puts("OK\n");
        fflush(stdout);
        Operation.clear();
    }
};

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
            MLog::print(robot[0]->orient_);

            Output::Print(Input::frameID);
        }
    }
}

// 刘智杰方案
namespace Solution1 {
    using namespace Input;
    using namespace Output;

    static constexpr int robot_num_ = 4;

    double dis_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
        return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

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

            for(int id = 0; id < 4; ++id) { // 枚举机器人
                if(robot[id] -> carry_id_) { // 携带物品
                    // 身边有 workbench
                    if(robot[id] -> workbench_) {
                        if(workbench[robot[id] -> workbench_] -> TryToSell(robot[id] -> carry_id_)) { // 可以卖出去手上的物品
                            Output::Sell(id);
                            continue ;
                        }
                    }

                    // 跑去卖手上的东西
                    int workbench_id = -1; double mn = 1e9;
                    for(int i = 0; i < K; ++i) { // 找一个最近的工作台
                        if(workbench[i] -> TryToSell(robot[id] -> carry_id_)) {
                            if(workbench_id == -1 || mn > dis_[id][i + robot_num_]) {
                                mn = dis_[id][i + robot_num_];
                                workbench_id = i;
                            }
                        }
                    }
                    if(workbench_id == -1) { // 找到有工作台
                        Output::Destroy(id);
                    } else { // 否则销毁手上的物件
                        double forward, rotate;
                        robot[id] -> ToPoint(workbench[workbench_id] -> x0_, workbench[workbench_id] -> y0_, forward, rotate);
                        Output::Forward(forward, rotate);
                    }
                } else { // 未携带物品
                    for(int id = 0; id < 4; ++id) {
                        // 身边有 workbench
                        if(robot[id] -> workbench_) {
                            int carry_id = robot[id] -> workbench_;
                            if(workbench[robot[id] -> workbench_] -> TryToBuy(carry_id)) { // 看看能不能买到物品
                                // 买之前先 check 有没有地方卖
                                bool can_sell = false;
                                for(int i = 0; i < K; ++i) {
                                    can_sell |= workbench[i] -> TryToSell(carry_id);
                                    if(can_sell) break;
                                }
                                if(can_sell) { // 以后可以卖的出去，买。
                                    Output::Buy(id);
                                    continue;
                                }
                            }
                        }

                        // 根据策略，选一个东西去买
                        double mn = 1e9; // 物品获利 / 距离
                        int carry_id = -1, workbench_buy, workbench_sell;
                        for(int k = 1; k <= 9; ++k) { // 枚举要买的物品
                            for(int i = 0; i < K; ++i) { // 从哪个工作站买
                                for(int j = 0; j < K; ++j) { // 从哪个工作站卖
                                    if(workbench[i] -> TryToBuy(k) && workbench[j] -> TryToSell(k)) {
                                        double money_per_distance = profit_[k] / (dis_[id][i + robot_num_] + dis_[i + robot_num_][j + robot_num_]);
                                        if(money_per_distance < mn) {
                                            mn = money_per_distance;
                                            carry_id = k;
                                            workbench_buy = i;
                                            workbench_sell = j;
                                        }
                                    }
                                }
                            }
                        }

                        if(mn < 1e9) { // 如果有则找到最优的策略，跑去买。
                            double forward, rotate;
                            robot[id] -> ToPoint(workbench[workbench_buy] -> x0_, workbench[workbench_buy] -> y0_, forward, rotate);
                            Output::Forward(forward, rotate);
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
