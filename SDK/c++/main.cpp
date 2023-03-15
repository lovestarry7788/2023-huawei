#include "log.h"
#include "geometry.h"
#include "input.h"
#include "output.h"
#include <cmath>
#include <algorithm>
#include <queue>


// 测试行走
namespace Solution2 {
    using namespace Input;
    using namespace Output;

    void Solve() {
        Input::ScanMap();
        using namespace Geometry;
        std::queue <Geometry::Point> route;
        // Geometry::Point{10,10}, Geometry::Point{40, 10}, Geometry::Point{40, 40}, Geometry::Point{10, 40}
        route.push(Geometry::Point{20, 20});
        route.push(Geometry::Point{25, 25});
        route.push(Geometry::Point{40, 30});
        route.push(Geometry::Point{10, 40});
        while (Input::ScanFrame()) {
            // Solution
            Geometry::Point loc{robot[0]->x0_, robot[0]->y0_};
            while (Geometry::Length(loc - route.front()) < 1e-1)
                route.pop();
            if (route.size()) {
                double forward = 0, rotate = 0;
                robot[0]->ToPoint(route.front().x, route.front().y, forward, rotate);
                robot[0]->ToPoint(route.front().x, route.front().y, forward, rotate);

                Output::Forward(0, forward);
                Output::Rotate(0, rotate);
            }

            // Log::print(Input::frameID);
            // for (auto i : Output::Operation)
            //     Log::print(i);
            Log::print(robot[0]->orient_);

            Output::Print(Input::frameID);
        }
    }
}


namespace Solution1 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;

    static constexpr int robot_num_ = 4;

    double forward[4], rotate[4];
    double dis_[110][110];
    double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};

    double Distance(double x0, double y0, double x1, double y1) {
        return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    void Solve() {
        Input::ScanMap();
        while (Input::ScanFrame()) {
            for (int idx = 0; idx < robot_num_ + K; ++idx) {
                for (int idy = 0; idy < robot_num_ + K; ++idy) {
                    double sx, sy, dx, dy;
                    if (idx < 4) {
                        sx = robot[idx]->x0_;
                        sy = robot[idx]->y0_;
                    } else {
                        sx = workbench[idx - robot_num_]->x0_;
                        sy = workbench[idx - robot_num_]->y0_;
                    }
                    if (idy < 4) {
                        dx = robot[idy]->x0_;
                        dy = robot[idy]->y0_;
                    } else {
                        dx = workbench[idy - robot_num_]->x0_;
                        dy = workbench[idy - robot_num_]->y0_;
                    }
                    dis_[idx][idy] = Distance(sx, sy, dx, dy);
                }
            }
            for (int id = 0; id < 4; ++id) { // 枚举机器人
                // Log::print("robot", id);
                if (robot[id]->carry_id_) { // 携带物品
                    // 身边有 workbench
                    if (robot[id]->workbench_ != -1) {
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
                    if (workbench_id == -1) { // 找不到工作台， 画一个大圆， 避免原地障碍
//                        Output::Forward(id, 4);
//                        Output::Rotate(id, pi / 4);
                        // Output::Destroy(id);
                    } else { // 否则销毁手上的物件
                        Log::print("try to sell, robot ", id, " carry_id ", robot[id]->carry_id_, " sell_workbench: ",
                                   workbench_id, " sell_workbench_type_id: ", workbench[workbench_id]->type_id_);
//                        double forward, rotate;
//                        robot[id]->ToPoint(workbench[workbench_id]->x0_, workbench[workbench_id]->y0_, forward[id],
                        robot[id]->ToPoint(workbench[workbench_id]->x0_, workbench[workbench_id]->y0_, forward[id],
                                           rotate[id]);
                        Output::Forward(id, forward[id]);
                        Output::Rotate(id, rotate[id]);
                    }
                } else { // 未携带物品
                    // 身边有 workbench
                    if (robot[id]->workbench_ != -1) {
                        int carry_id = workbench[robot[id]->workbench_]->type_id_;
                        // Log::print("Can buy, id: ", id, " workbench : ", robot[id] -> workbench_, " type_id: ", workbench[robot[id] -> workbench_] -> type_id_);
                        if (workbench[robot[id]->workbench_]->TryToBuy(carry_id)) { // 看看能不能买到物品
                            // 买之前先 check 有没有地方卖
                            bool can_sell = false;
                            for (int i = 0; i < K; ++i) {
                                can_sell |= workbench[i]->TryToSell(carry_id);
                                if (can_sell) break;
                            }

//                             // 检测会不会和其它机器人碰撞
//                             bool crashed = false;
//                             for(int i = 0; i < 4; ++i) if(i != id) {
//                                 double dis = (robot[i] -> carry_id_ > 0) ? 0.53 : 0.45;
//                                 double x_ = robot[id] -> x0_ - robot[i] -> x0_;
//                                 double y_ = robot[id] -> y0_ - robot[i] -> y0_;
//                                 if(dis + 0.53 >= sqrt(x_ * x_ + y_ * y_)) crashed = true;
//                             }

                            if (can_sell) { // 以后可以卖的出去，买。
                                Log::print("buy!, id: ", id, " workbench : ", robot[id]->workbench_, " can_sell : ",
                                           can_sell);
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
                                if (workbench[i]->TryToBuy(k) && workbench[j]->TryToSell(k)) {
                                    double money_per_distance = profit_[k] / (dis_[id][i + robot_num_] +
                                                                              dis_[i + robot_num_][j + robot_num_]);
                                    // Log::print("things: ", k," buy from : ", i," sell from: ", j , " money_per_distance: ", money_per_distance);
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

                    Log::print("choose to buy, Robot ", id, " : ", carry_id, workbench_buy, workbench_sell);


                    if (fabs(mn - 0) > 1e-5) { // 如果有则找到最优的策略，跑去买。
//                         double forward, rotate;
//                        robot[id]->ToPoint(workbench[workbench_buy]->x0_, workbench[workbench_buy]->y0_, forward[id],
                        robot[id]->ToPoint(workbench[workbench_buy]->x0_, workbench[workbench_buy]->y0_, forward[id],
                                           rotate[id]);
                        Output::Forward(id, forward[id]);
                        Output::Rotate(id, rotate[id]);
                    }
                }
            }


            Output::Print(Input::frameID);
            Log::print("frame_id: ", frameID, " OK! ");
        }
    }
}

//template<typename T>
//struct EK {
//    const T INF = 2e18;
//    T cost;
//    struct Edge {
//        int to, nxt, w, f;
//    };
//    vector <Edge> e;
//    vector<bool> vis;
//    vector<int> head, lst;
//    vector <T> dis;
//    int tot, s, t;
//
//    EK(int n, int m, int st, int ed) {
//        head.assign(n + 1, -1);
//        e.resize(m << 1);
//        dis.resize(n + 1);
//        vis.resize(n + 1);
//        s = st;
//        t = ed;
//        tot = 0;
//        cost = 0;
//    }
//
//    inline int add(int u, int v, int w, int f) {
//        e[tot] = (Edge) {v, head[u], w, f};
//        head[u] = tot++;
//        return tot - 1;
//    }
//
//    inline int insert(int u, int v, int w, int f) {
//        int tmp = add(u, v, w, f);
//        add(v, u, -w, 0);
//        return tmp;
//    }
//
//    inline bool spfa() {
//        queue<int> q;
//        dis.assign(dis.size(), INF);
//        vis.assign(vis.size(), false);
//        dis[s] = 0;
//        vis[s] = 1;
//        lst = head;
//        q.push(s);
//        while (!q.empty()) {
//            int u = q.front();
//            q.pop();
//            for (int i = head[u]; ~i; i = e[i].nxt) {
//                int v = e[i].to;
//                if (e[i].f > 0 && dis[v] > dis[u] + e[i].w) {
//                    dis[v] = dis[u] + e[i].w;
//                    if (!vis[v]) {
//                        vis[v] = 1;
//                        q.push(v);
//                    }
//                }
//            }
//            vis[u] = 0;
//        }
//        return dis[t] != INF;
//    }
//
//    int dfs(int u, int f) {
//        if (u == t || !f) return f;
//        vis[u] = 1;
//        int ans = 0;
//        for (int &i = lst[u]; ~i; i = e[i].nxt) {
//            int v = e[i].to;
//            if (e[i].f && dis[v] == dis[u] + e[i].w && !vis[v]) {
//                int val = dfs(v, min(e[i].f, f - ans));
//                if (val) {
//                    e[i].f -= val;
//                    e[i ^ 1].f += val;
//                    cost += val * e[i].w;
//                    ans += val;
//                    if (ans == f) break;
//                }
//            }
//        }
//        vis[u] = 0;
//        return ans;
//    }
//
//    inline array<T, 2> work() {
//        T ans = 0;
//        while (spfa()) {
//            T d = dfs(s, INF);
//            ans += d;
//        }
//        return {ans, cost};
//    }
//};

namespace Solution_zn_1 {
    using namespace Input;
    using namespace Output;
    using namespace Geometry;

    static constexpr int robot_num_ = 4;

    double forward[4], rotate[4];
    double dis_[110][110];
    const int material[] = {0, 0, 0, 0, 2 + 4, 2 + 8, 4 + 8, 112, 128, 255};
    const double profit_[8] = {0, 3000, 3200, 3400, 7100, 7800, 8300, 29000};
    int goal_sell[4] = {-1, -1, -1, -1};
    int goal_buy[4] = {-1, -1, -1, -1};//目标工作台（购买的， 售卖的）
    int vis_buy[50] = {0}, vis_sell[50] = {0};
    int pocess_sell[50] = {0}, pocess_buy[50] = {0};
    //买东西看product_status, 卖东西看materials_status, 这个值是可以预估
    //想开一个二维数组了， pocess[time][workbench_id]表示在第time帧，workbench_id是否可用
    //目前先写一个跟time帧无关的

    double Distance(double x0, double y0, double x1, double y1) {
        return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    }

    inline int period() {
        bool flag_456 = false;
        bool flag_7 = false;
        for (int i = 0; i < K; i++) {
            //首先看456有没有填满，有的话是一阶段
            //如果456填满了考虑卖到7/9当二阶段
            //如果7/9填满了就考虑把7卖到9当三阶段
            //如果卖不出去就不用管了

            flag_456 |= (workbench[i]->type_id_ <= 6 && workbench[i]->type_id_ >= 4 &&
                         pocess_sell[i]);
            flag_7 |= (workbench[i]->type_id_ == 7 && pocess_sell[i]);

            //有4～6的工作台且可以卖
            //1 2 3共享资源
            //4 5 6独享
            //专机送4 5 6 to 7/9

        }

        if (flag_456 == true) return 1;
        else if (flag_7 == true) return 2;
        else return 3;

    }

    inline void period1_solve(int id) {
        //买123， 卖456

        Log::print("Robot_id: ", id, "goal_buy: ", goal_buy[id], "goal_sell: ", goal_sell[id], '\n');
        //购买123
        //售出456
        //for (int id = 0; id < 4; id++) {


        if (robot[id]->workbench_ != -1) {
            //当前机器人位于某个操作台
            //可以考虑在这里购买或者售卖

            int workbench_id = robot[id]->workbench_;

            if (robot[id]->carry_id_ && robot[id]->workbench_ == goal_sell[id]) {

                Output::Sell(id);
                goal_sell[id] = -1;
                vis_sell[workbench_id] = false;

            } else if (robot[id]->carry_id_ == 0 && robot[id]->workbench_ == goal_buy[id]) {

                //因为购买的是123， 对结果没有影响， 所以对于123的pocess_buy全都设为false即可
                Output::Buy(id);
                goal_buy[id] = -1;
                vis_buy[workbench_id] = false;
                pocess_buy[workbench_id] = workbench[workbench_id]->product_status_;

            }
        }

//        if (goal_buy[id] != -1) {
//
//            //已经有确定的目标了， 就坚持跑到那里
//
//            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
//            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
//            //        pocess_sell[goal_id] ^= 1 << robot[id]->carry_id_;
//            Output::Forward(id, forward[id]);
//            Output::Rotate(id, rotate[id]);
//
//            return;
//        }
//
        if (goal_buy[id] == -1 && goal_sell[id] != -1) {
            //已经买过并且
            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);
            return;

        }

        //没带着东西
        //找一个近的， 123中的去买, 但要有的卖
        double mn;
        if(goal_buy[id] != -1)
            mn = Distance(robot[id]->x0_, robot[id]->y0_, workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_) +
                    Distance(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, workbench[goal_sell[id]]->x0_,
                             workbench[goal_sell[id]]->y0_);
        else
            mn = 2e9;
        int goal_carry_id = -1;
        for (int carry_id = 1; carry_id <= 3; carry_id++) {//要购买的物品
            for (int i = 0; i < K; i++) {
                if (workbench[i]->TryToBuy(carry_id) && !vis_buy[i]) {
                    for (int j = 0; j < K; j++) {
                        if (workbench[j]->type_id_ <= 6 && workbench[j]->type_id_ >= 4 && workbench[j]->TryToSell(carry_id) && ((pocess_sell[j] >> carry_id) & 1) && pocess_sell[j]) {
                            double dist = Distance(robot[id]->x0_, robot[id]->y0_, workbench[i]->x0_, workbench[i]->y0_) +
                                          Distance(workbench[i]->x0_, workbench[i]->y0_, workbench[j]->x0_,
                                                   workbench[j]->y0_);
                            //Log::print("Robot id: ", id, "find i, j: ", i, j, dist, mn);
                            if (mn > dist) {
                                mn = dist;
                                goal_buy[id] = i;
                                goal_sell[id] = j;
                                goal_carry_id = carry_id;
                                //Log::print("Robot id: ", id, "buy: ", i, "sell: ", j, "carry_id", carry_id);
                            }
                        }
                    }
                }
            }
        }
        if (goal_buy[id] != -1) {
            //可以
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            pocess_sell[goal_sell[id]] ^= 1 << goal_carry_id;
            pocess_buy[goal_buy[id]] = 0;
            vis_buy[goal_buy[id]] = true;
            vis_sell[goal_sell[id]] = true;
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);
        }
    }

    inline void period2_solve(int id) {
        //买4 5 6， 卖7

        Log::print("Robot_id: ", id, "goal_buy: ", goal_buy[id], "goal_sell: ", goal_sell[id], '\n');
        //购买456
        //售出7
        //for (int id = 0; id < 4; id++) {


        if (robot[id]->workbench_ != -1) {
            //当前机器人位于某个操作台
            //可以考虑在这里购买或者售卖

            int workbench_id = robot[id]->workbench_;

            if (robot[id]->carry_id_ && robot[id]->workbench_ == goal_sell[id]) {

                Output::Sell(id);
                goal_sell[id] = -1;
                vis_sell[workbench_id] = false;

            } else if (robot[id]->carry_id_ == 0 && robot[id]->workbench_ == goal_buy[id]) {

                //购买456，会
                Output::Buy(id);
                goal_buy[id] = -1;
                vis_buy[workbench_id] = false;
                pocess_buy[workbench_id] = workbench[workbench_id]->product_status_;

            }
        }

        if (goal_buy[id] != -1) {

            //已经有确定的目标了， 就坚持跑到那里

            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            //        pocess_sell[goal_id] ^= 1 << robot[id]->carry_id_;
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);

            return;
        }

        if (goal_sell[id] != -1) {

            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);

            return;

        }

        //没带着东西
        //找一个近的， 456中的去买, 但要有的卖
        double mn = 1e9;
        int goal_carry_id = -1;
        for (int carry_id = 4; carry_id <= 6; carry_id++) {//要购买的物品
            for (int i = 0; i < K; i++) {
                if (workbench[i]->TryToBuy(carry_id) && (!vis_buy[i] || (goal_buy[id] == i))) {
                    for (int j = 0; j < K; j++) {
                        if ((workbench[j]->type_id_ == 7 || workbench[j]->type_id_ == 9) && workbench[j]->TryToSell(carry_id) && ((pocess_sell[j] >> carry_id) & 1) && pocess_sell[j]) {
                            double dist = Distance(robot[id]->x0_, robot[id]->y0_, workbench[i]->x0_, workbench[i]->y0_) +
                                          Distance(workbench[i]->x0_, workbench[i]->y0_, workbench[j]->x0_,
                                                    workbench[j]->y0_);
                            //Log::print("Robot id: ", id, "find i, j: ", i, j, dist, mn);
                            if (mn > dist) {
                                mn = dist;
                                goal_buy[id] = i;
                                goal_sell[id] = j;
                                goal_carry_id = carry_id;
                                //Log::print("Robot id: ", id, "buy: ", i, "sell: ", j, "carry_id", carry_id);
                            }
                        }
                    }
                }
            }
        }
        if (goal_buy[id] != -1 /*&& goal_carry_id != -1*/) {
            //可以
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            pocess_sell[goal_sell[id]] ^= 1 << goal_carry_id;
            pocess_buy[goal_buy[id]] = 0;
            vis_buy[goal_buy[id]] = true;
            vis_sell[goal_sell[id]] = true;
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);
        }
    }

    inline void period3_solve(int id) {
        //买7， 卖8/9

        Log::print("Robot_id: ", id, "goal_buy: ", goal_buy[id], "goal_sell: ", goal_sell[id], '\n');
        //购买123
        //售出456
        //for (int id = 0; id < 4; id++) {


        if (robot[id]->workbench_ != -1) {
            //当前机器人位于某个操作台
            //可以考虑在这里购买或者售卖

            int workbench_id = robot[id]->workbench_;

            if (robot[id]->carry_id_ && robot[id]->workbench_ == goal_sell[id]) {

                Output::Sell(id);
                goal_sell[id] = -1;
                vis_sell[workbench_id] = false;

            } else if (robot[id]->carry_id_ == 0 && robot[id]->workbench_ == goal_buy[id]) {

                //因为购买的是123， 对结果没有影响， 所以对于123的pocess_buy全都设为false即可
                Output::Buy(id);
                goal_buy[id] = -1;
                vis_buy[workbench_id] = false;//更新为false， 能否购买由product_status决定
                pocess_buy[workbench_id] = workbench[workbench_id]->product_status_;

            }
        }

        if (goal_buy[id] != -1) {

            //已经有确定的目标了， 就坚持跑到那里

            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            //        pocess_sell[goal_id] ^= 1 << robot[id]->carry_id_;
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);

            return;
        }

        if (robot[id]->carry_id_) {

            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_sell[id]]->x0_, workbench[goal_sell[id]]->y0_, forward[id], rotate[id]);
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);

            return;

        }

        //没带着东西
        //找一个近的， 123中的去买, 但要有的卖
        double mn = 1e9;
        int goal_carry_id = -1;
        for (int carry_id = 7; carry_id <= 7; carry_id++) {//要购买的物品
            for (int i = 0; i < K; i++) {
                if (workbench[i]->TryToBuy(carry_id) && !vis_buy[i]) {
                    for (int j = 0; j < K; j++) {
                        if (workbench[j]->type_id_ <= 9 && workbench[j]->type_id_ >= 8 && workbench[j]->TryToSell(carry_id) && ((pocess_sell[j] >> carry_id) & 1) && pocess_sell[j]) {
                            double dist = Distance(robot[id]->x0_, robot[id]->y0_, workbench[i]->x0_, workbench[i]->y0_) +
                                          Distance(workbench[i]->x0_, workbench[i]->y0_, workbench[j]->x0_,
                                                   workbench[j]->y0_);
                            //Log::print("Robot id: ", id, "find i, j: ", i, j, dist, mn);
                            if (mn > dist) {
                                mn = dist;
                                if(goal_buy[id] != -1) {
                                    pocess_sell[goal_sell[id]] ^= 1 << goal_carry_id;
                                    pocess_buy[goal_buy[id]] ^= 1;
                                    vis_buy[goal_buy[id]] ^= 1;
                                    vis_sell[goal_sell[id]] ^= 1;
                                }
                                goal_buy[id] = i;
                                goal_sell[id] = j;
                                pocess_sell[goal_sell[id]] ^= 1 << goal_carry_id;
                                pocess_buy[goal_buy[id]] ^= 1;
                                vis_buy[goal_buy[id]] ^= 1;
                                vis_sell[goal_sell[id]] ^= 1;
                                goal_carry_id = carry_id;
                                //Log::print("Robot id: ", id, "buy: ", i, "sell: ", j, "carry_id", carry_id);
                            }
                        }
                    }
                }
            }
        }
        if (goal_buy[id] != -1 /*&& goal_carry_id != -1*/) {
            //可以
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            robot[id]->ToPoint(workbench[goal_buy[id]]->x0_, workbench[goal_buy[id]]->y0_, forward[id], rotate[id]);
            Output::Forward(id, forward[id]);
            Output::Rotate(id, rotate[id]);
        }
    }

    void Solve() {
        Input::ScanMap();

        //EK Robot_SearchRode;//感觉需要一些转换， 来跑出一个近似度高的解

        while (Input::ScanFrame()) {

            for(int i = 0; i < K; i++) {
                if(!vis_buy[i]) pocess_buy[i] = workbench[i]->product_status_;
                if(!vis_sell[i]) pocess_sell[i] = (workbench[i]->materials_status_ ^ material[workbench[i]->type_id_]);
                //初始化， 如果没有被选为目标， 那么产品是怎样的
//            if(!vis_sell[i]) pocess_sell[i] = material[workbench[i]->type_id_];
//            Log::print("Workbench_id: ", i, "which can buy: ", pocess_buy[i] * workbench[i]->type_id_, "which can sell: ", pocess_sell[i]);
            }

            for (int id = 0; id < 4; id++) {
                period1_solve(id);
//                int period_ = period();
//                Log::print("robot_id: ", id, "period: ", period_, '\n');
//                if (period_ == 1) period1_solve(id);
//                else if (period_ == 2) period2_solve(id);
//                else period3_solve(id);
            }

            Output::Print(Input::frameID);
            Log::print("frame_id: ", frameID, " OK! ");
        }
    }

}

int main() {
    Solution_zn_1::Solve();
    return 0;
}
