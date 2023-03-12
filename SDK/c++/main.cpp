#include "robot.h"
#include "workbench.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string_view>

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
                workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
            }

            robot.resize(4);
            for(int id = 0; id < 4; ++id) {
                int workbench_id, carry_id;
                double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
                scanf("%d%d%lf%lf%lf%lf%lf%lf%lf",&workbench_id, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                      &orient, &x0, &y0);
                robot[id] = std::make_shared<Robot>(id, workbench_id, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
            }

            readUntilOK();
            return true;
        }
        return false;
    }
};

namespace Output {
    std::vector<std::string_view> Operation;

    void Forward(int robot_id, double forward) {
        Operation.emplace_back(std::string_view{"forward " + std::to_string(robot_id) + " " + std::to_string(forward)});
    }

    void Rotate(int robot_id, double rotate) {
        Operation.emplace_back(std::string_view{"rotate " + std::to_string(robot_id) + " " + std::to_string(rotate)});
    }

    void Buy(int robot_id) {
        Operation.emplace_back(std::string_view{"buy " + std::to_string(robot_id)});
    }

    void Sell(int robot_id) {
        Operation.emplace_back(std::string_view{"sell " + std::to_string(robot_id)});
    }

    void Destroy(int robot_id) {
        Operation.emplace_back(std::string_view{"destory " + std::to_string(robot_id)});
    }

    void Print(int frame_id) {
        printf("%d\n", frame_id);
        for(const auto& u: Operation) {
            printf("%s", u.data());
        }
        puts("OK\n");
        fflush(stdout);
    }
};

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
    Solution1::Solve();
    return 0;
}
