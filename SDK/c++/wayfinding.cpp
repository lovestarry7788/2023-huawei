#include "wayfinding.h"
#include "geometry.h"
#include "input.h"
#include "log.h"
#include <algorithm>
#include <queue>
#include <map>
#include <vector>
#include <utility>
#include <iostream>

using namespace Geometry;
using namespace Input;
using namespace WayFinding2;


std::vector<std::array<double, 2>> WayFinding2::PointInBlock[2][100][100];
int WayFinding2::Edge_Num[2];//建边， 边的数量
int WayFinding2::Head[2][N_];//建边, 领接表头指针
Edge WayFinding2::Edge_[2][M_];
Geometry::Point WayFinding2::Point_[2][60000];//编号对应的点的坐标
std::vector<double> WayFinding2::Unique_x;//用于离散化x坐标
std::vector<double> WayFinding2::Unique_y;//用于离散化y坐标
std::vector<double> WayFinding2::joint_obs_[101], WayFinding2::joint_obs_x0_;
double WayFinding2::Dis[2][400][N_];//最短路： 初始建图的最短路
//double WayFinding2::Dis2[2][450][N_];//最短路2：
int WayFinding2::From[2][400][N_][Binary_Limit];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点

bool WayFinding2::Check_Avoid_Crash_Wall(int o, int A, int B) { //测试沿着AB连线走会不会撞墙
    double x0, x1, y0, y1;
    int x0_, x1_, y0_, y1_;
    double d;
    auto a = Point_[o][A];
    auto b = Point_[o][B]; // 找到 a, b 之间的路径
    bool valid = true;
    double mind = 1e18;
    double radius = Radius[o];
    x0 = std::min(a.x, b.x) - radius, x1 = std::max(a.x, b.x) + radius, y0 = std::min(a.y, b.y) - radius, y1 = std::max(a.y, b.y) + radius;
    if(A == 3679 && B == 3921)
        Log::print(y0, y1);
    x0_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x0) - joint_obs_x0_.begin() - 1; x0_ = std::max(x0_, 0);
    x1_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x1) - joint_obs_x0_.begin();
    for (int x = x0_; x <= x1_; ++x) if(!joint_obs_[x].empty()){
            y0_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y0) - joint_obs_[x].begin() - 1; y0_ = std::max(y0_, 0);
            y1_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y1) - joint_obs_[x].begin();
            for (int y = y0_; y <= y1_; ++y) {
                d = DistanceToSegment({joint_obs_x0_[x], joint_obs_[x][y]}, a, b);
                if(A == 3679 && B == 3921)
//                    Log::print("Block: ", joint_obs_x0_[x], joint_obs_[x][y]),
                    Log::print("Radius: ", Radius[o], "i: ", A, "j: ", B, "a.x: ", a.x, "a.y: ", a.y, "b.x: ", b.x, "b.y: ",b.y, "x0: ", joint_obs_x0_[x], "y0: ", joint_obs_[x][y], "d: ", d);
                mind = std::min(mind, d);
                // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
                if (mind < radius + 2e-2) { // 操作误差
                    valid = false;
                    break;
                }
            }
            if(!valid) break;
        }
    if(A == 3679 && B == 3921)
        Log::print("Radius: ", Radius[o], "i: ", A, "j: ", B, "a.x: ", a.x, "a.y: ", a.y, "b.x: ", b.x, "b.y: ",b.y,  "d: ", mind);
    return valid;
}//测试沿着AB连线走会不会撞墙

int WayFinding2::FindRoute(int o, int A, int B) {//A是任意一个点， B是一个工作台
    int P = A;
//    return From[o][B][A][0];
    for(int i = Binary_Limit - 1; ~i; i--) {
        if(Check_Avoid_Crash_Wall(o, From[o][B][P][i], A))
            P = From[o][B][P][i];
    }
    return A == P ? -1 : P;//表示没有找到P点
}

template<typename T> void WayFinding2::Unique(std::vector<T> &a){//离散化a
    sort(a.begin(), a.end());
    a.erase(unique(a.begin(), a.end()), a.end());
}

std::map<std::array<int,2>, int> WayFinding2::map_id;
int WayFinding2::GetID(int o, double x_, double y_) {
    int x0_ = std::lower_bound(Unique_x.begin(), Unique_x.end(), x_) - Unique_x.begin();
    if(fabs(Unique_x[x0_] - x_) < eps || x0_ == 0); else x0_--;
    int y0_ = std::lower_bound(Unique_y.begin(), Unique_y.end(), y_) - Unique_y.begin();
    if(fabs(Unique_y[y0_] - y_) < eps || y0_ == 0); else y0_--;
    auto now = map_id.find({x0_, y0_});
    return now != map_id.end() ? now->second : -1;
}//获得x,y对应的离散化id

void WayFinding2::Insert_Edge(int o, int u, int v, double dis) {
    Edge_[o][Edge_Num[o]] = (Edge){u, v, Head[o][u], dis};
    Head[o][u] = Edge_Num[o]++;
}

void WayFinding2::Init() {
    memset(Head, -1, sizeof(Head));
    memset(From, -1, sizeof(From));
    map_id.clear();
    int cnt_robot = 0, cnt_workbench = 0;
    Edge_Num[0] = Edge_Num[1] = 0;
    std::vector<int> workbench_pos;
    for (int i = 0; i < map_size_; i++)
        for (int j = 0; j < map_size_; j++) {
            double px = j * 0.5 + 0.25;
            double py = (map_size_ - i - 1) * 0.5 + 0.25;
            if(map_[i][j] == '#') {
                Log::print("Block: ", i, j, px, py);
                continue;//障碍的话不需要建点
            }
            if(map_[i][j] == '.' || map_[i][j] == 'A') {
                Unique_x.push_back(px);
                Unique_y.push_back(py);
                auto [i2, j2] = GetBlockID(px, py);
                PointInBlock[0][i2][j2].push_back({px, py});
                PointInBlock[1][i2][j2].push_back({px, py});
                continue;
            }
//            cnt_workbench++;
            for(int o = 0; o < 2; o++) {//两种图判一下建边
                for(int k = 0; k < 9; k++) {
                    double dx = direction_x[o][k];
                    double dy = direction_y[o][k];
                    double nx = px + dx, ny = py + dy;
                    //无脑离散化， 只要不建边就是不合法的点
                    Unique_x.push_back(nx);
                    Unique_y.push_back(ny);
                    auto [i2, j2] = GetBlockID(nx, ny);
                    PointInBlock[0][i2][j2].push_back({nx, ny});
                    PointInBlock[1][i2][j2].push_back({nx, ny});
                }
            }
        }
    Unique(Unique_x);
    Unique(Unique_y);
    //初始化建图， 先将所有的点离散化之后再开始建边

    int total = 450;
    for (int i = 0; i < map_size_; i++)
        for (int j = 0; j < map_size_; j++){
            for(int o = 0; o < 2; o++){
                Unique(PointInBlock[o][i][j]);
                for(auto [x_, y_]: PointInBlock[o][i][j]){
                    int x0_ = std::lower_bound(Unique_x.begin(), Unique_x.end(), x_) - Unique_x.begin();
                    if(fabs(Unique_x[x0_] - x_) < eps || x0_ == 0); else x0_--;
                    int y0_ = std::lower_bound(Unique_y.begin(), Unique_y.end(), y_) - Unique_y.begin();
                    if(fabs(Unique_y[y0_] - y_) < eps || y0_ == 0); else y0_--;
                    auto it = map_id.find({x0_, y0_});
                    if(it == map_id.end()){
                        Point_[o][total] = {x_, y_};
                        map_id[{x0_, y0_}] = total++;
                    }
                }
            }
        }

    Log::print(total);

    int total_workbench = 0;
    for(int i = 0; i < map_size_; ++i) {
        for(int j = 0; j < map_size_; ++j) {
            if(map_[i][j] == '#') {
                double px = j * 0.5 + 0.25;
                joint_obs_x0_.push_back(px + 0.25);
                joint_obs_x0_.push_back(px - 0.25);
                continue;
            }//是障碍做拓展
            if(map_[i][j] == 'A' || map_[i][j] == '.') continue;
            if(map_[i][j] >= '1' && map_[i][j] <= '9') {
                double px = j * 0.5 + 0.25;
                double py = (map_size_ - i - 1) * 0.5 + 0.25;
                for(int o = 0; o < 2; o++){
                    for(int k = 0; k < 9; k++) {
                        double x_ = px + direction_x[o][k];
                        double y_ = py + direction_y[o][k];
                        int x0_ = std::lower_bound(Unique_x.begin(), Unique_x.end(), x_) - Unique_x.begin();
                        if(fabs(Unique_x[x0_] - x_) < eps || x0_ == 0); else x0_--;
                        int y0_ = std::lower_bound(Unique_y.begin(), Unique_y.end(), y_) - Unique_y.begin();
                        if(fabs(Unique_y[y0_] - y_) < eps || y0_ == 0); else y0_--;
                        auto now = map_id.find({x0_, y0_});
                        if(now->second >= 450) {
                            Point_[o][total_workbench] = {x_, y_};
                            workbench_pos.push_back(total_workbench);
                            map_id[{x0_, y0_}] = total_workbench++;
                        }
//                        Log::print(i, " ", j, " ", nx, " ", ny, " ", ID);
                    }
                }
            }//是工作台记编号
        }
    }

    Unique(workbench_pos);
    Log::print(workbench_pos.size());
    for(int s: workbench_pos) Log::print(s);

    /*
     * 对所有的障碍点集进行去重
     */

//    for (int i = 0; i < map_size_; i++)
//        for (int j = 0; j < map_size_; j++)
//            for(auto[x_, y_]: PointInBlock[0][i][j])
//                Log::print(i, " ", j, " ", x_, " ", y_, " ", GetID(0, x_, y_));

    Unique(joint_obs_x0_);
    for (int i = 0; i < map_size_; i++) {
        for (int j = 0; j < map_size_; j++)
            if(map_[i][j] == '#'){
                double px = j * 0.5 + 0.25;
                double py = (map_size_ - i - 1) * 0.5 + 0.25;
                int idx;
                idx = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), px + 0.25) - joint_obs_x0_.begin();
                joint_obs_[idx].push_back(py + 0.25);
                joint_obs_[idx].push_back(py - 0.25);

                idx = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), px - 0.25) - joint_obs_x0_.begin();
                joint_obs_[idx].push_back(py + 0.25);
                joint_obs_[idx].push_back(py - 0.25);
            }
    }

    for(int i = 0; i < joint_obs_x0_.size(); ++i) {
        Unique(joint_obs_[i]);
    }
//
//    Log::print(Unique_x.size());
//    Log::print(Unique_y.size());
//
    for(int o = 0; o < 2; o++){
        for(int i = 0; i < map_size_; ++i) {
            for(int j = 0; j < map_size_; ++j) {
                for(int k = 0; k < 8; k++) {
                    int i2 = i + dx[k], j2 = j + dy[k];
                    if(i2 < 0 || i2 >= map_size_ || j2 < 0 || j2 >= map_size_)
                        continue;
                    for(auto [x_, y_]: PointInBlock[o][i][j]){
                        int x0_ = std::lower_bound(Unique_x.begin(), Unique_x.end(), x_) - Unique_x.begin();
                        if(fabs(Unique_x[x0_] - x_) < eps || x0_ == 0); else x0_--;
                        int y0_ = std::lower_bound(Unique_y.begin(), Unique_y.end(), y_) - Unique_y.begin();
                        if(fabs(Unique_y[y0_] - y_) < eps || y0_ == 0); else y0_--;
                        int id1 = map_id[{x0_, y0_}];
                        for(auto [x_1, y_1]: PointInBlock[o][i2][j2]) {
                            int x1_ = std::lower_bound(Unique_x.begin(), Unique_x.end(), x_1) - Unique_x.begin();
                            if(fabs(Unique_x[x1_] - x_1) < eps || x1_ == 0); else x1_--;
                            int y1_ = std::lower_bound(Unique_y.begin(), Unique_y.end(), y_1) - Unique_y.begin();
                            if(fabs(Unique_y[y1_] - y_1) < eps || y1_ == 0); else y1_--;
                            int id2 = map_id[{x1_, y1_}];
                            if(Check_Avoid_Crash_Wall(o, id1, id2)) {
                                double dist = Geometry::Dist(x0_, y0_, x1_, y1_);
                                Insert_Edge(o, id1, id2, dist);
//                                Log::print("Edge from:", id1, " to ", id2);
                            }
                        }
                    }
                }
//                double px_1 = j * 0.5 + 0.25;
//                double py_1 = (map_size_ - i - 1) * 0.5 + 0.25;
//                auto [i2, j2] = GetBlockID(px_1, py_1);
//                PointInBlock[i2][j2].push_back(GetID(o, px_1, py_1));
                //当前格子和上下左右斜着8个格子
            }
        }
    }
    Log::print("Edges Connected Complete!");
//    Unique(workbench_pos);
//    Log::print(workbench_pos.size());
//    for(int s: workbench_pos) Log::print(s);
//    for(int o = 0; o < 2; o++){
//        for(int u = 0; u < total; u++){
//            for (int k = Head[o][u]; k != -1; k = Edge_[o][k].next) {
//                int v = Edge_[o][k].to;
//                Log::print("u: ", u, "v: ", v);
//            }
//        }
//    }
    Log::print(total);
    Log::print("0: ", Edge_Num[0], " 1: ", Edge_Num[1]);
//
////    Log::print(time(0), "ms");
//
    for(int o = 0; o < 2; ++o) {
        for (int s: workbench_pos) {
            int cnt = 0;
            Log::print(s);
            Dijkstra(o, s, cnt);
            Log::print(s, " ", cnt);
            for (int j = 1; j < Binary_Limit; ++j) {
                for (int i = 0; i < N_; ++i) {
//                    Log::print("beizeng: ", o, s, i, j-1);
                    if (From[o][s][i][j - 1] >= 0 && From[o][s][i][j - 1] < total)
                        From[o][s][i][j] = From[o][s][From[o][s][i][j - 1]][j - 1];
                    else
                        From[o][s][i][j] = -1;
                }
            }
        }
    }
    Log::print("FindRoute Complete!");
}

std::array<int,2> WayFinding2::GetBlockID(double x0_, double y0_) {
    int j = floor(x0_ / 0.5);
    int i = map_size_ - floor(y0_ / 0.5) - 1;
    return {i, j};
}

std::priority_queue<Status> Q;
void WayFinding2::Dijkstra(int o, int s, int &total_time) { //有没有拿东西o:0/1, 起点： 工作台
    std::vector<bool> vis(N_, false);
    for(int i = 0; i < N_; ++i) {
//        From[o][s][i][0] = -1;
        Dis[o][s][i] = INF;
    }
    Dis[o][s][s] = 0.0;
    Log::print(s, "?");
    while(!Q.empty()) Q.pop();
    Q.push(Status{s, Dis[o][s][s]});
    while (!Q.empty()) {
//        total_time++;
//        if(total_time > 10000000) break;
//        Log::print("from: ", s, total_time);
        auto x = Q.top(); Q.pop();
        int u = x.u;
        if (vis[u]) continue;
        vis[u] = true;
        for (int k = Head[o][u]; k != -1; k = Edge_[o][k].next) {
//             Log::print("o: ", o, "s: ", s, "u: ", u, "k: ", k);
//             Log::print("from: ", Edge_[o][k].from, " to ", Edge_[o][k].to);
            if (Dis[o][s][Edge_[o][k].to] - (Dis[o][s][Edge_[o][k].from] + Edge_[o][k].dis) > eps) {
                Dis[o][s][Edge_[o][k].to] = Dis[o][s][Edge_[o][k].from] + Edge_[o][k].dis;
                From[o][s][Edge_[o][k].to][0] = Edge_[o][k].from; // 前驱点
                Q.push(Status{Edge_[o][k].to, Dis[o][s][Edge_[o][k].to]});
            }
        }
    }
    std::vector<bool>{}.swap(vis);
}
