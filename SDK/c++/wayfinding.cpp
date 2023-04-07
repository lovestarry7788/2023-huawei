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

int WayFinding2::Edge_Num[2];//建边， 边的数量
int WayFinding2::Head[2][N_];//建边, 领接表头指针
Edge WayFinding2::Edge_[2][M_];
Geometry::Point WayFinding2::Point_[2][N_];//编号对应的点的坐标
std::vector<double> WayFinding2::Unique_x[2];//用于离散化x坐标
std::vector<double> WayFinding2::Unique_y[2];//用于离散化y坐标
std::vector<double> WayFinding2::joint_obs_[101], WayFinding2::joint_obs_x0_;
double WayFinding2::Dis[2][50][N_];//最短路： 初始建图的最短路
double WayFinding2::Dis2[2][50][N_];//最短路2：
int WayFinding2::From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点

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
    x0_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x0) - joint_obs_x0_.begin() - 1; x0_ = std::max(x0_, 0);
    x1_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x1) - joint_obs_x0_.begin();
    for (int x = x0_; x <= x1_; ++x) if(!joint_obs_[x].empty()){
            y0_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y0) - joint_obs_[x].begin() - 1; y0_ = std::max(y0_, 0);
            y1_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y1) - joint_obs_[x].begin();
            for (int y = y0_; y <= y1_; ++y) {
                d = DistanceToSegment({joint_obs_x0_[x], joint_obs_[x][y]}, a, b);
                //                        if(j == 71) Log::print("Radius: ", Radius[o], "i: ", i, "j: ", j, "a.x: ", a.x, "a.y: ", a.y, "b.x: ", b.x, "b.y: ",b.y, "x0: ",joint_obs_x0_[x], "y0: ", joint_obs_[x][y], "d: ", d);
                mind = std::min(mind, d);
                // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
                if (d < radius + 2e-2) { // 操作误差
                    valid = false;
                    break;
                }
            }
            if(!valid) break;
        }
    return valid;
}//测试沿着AB连线走会不会撞墙

int WayFinding2::FindRoute(int o, int A, int B) {//A是任意一个点， B是一个工作台
    int P = A;
    for(int i = 14; ~i; i--) {
        if(Check_Avoid_Crash_Wall(o, From[o][B][P][i], A))
            P = From[o][B][P][i];
    }
    return A == P ? -1 : P;//表示没有找到P点
}

void WayFinding2::Unique(std::vector<double> &a){//离散化a
    sort(a.begin(), a.end());
    a.erase(unique(a.begin(), a.end()), a.end());
}

int WayFinding2::map_id[N_xy][N_xy];
int WayFinding2::GetID(int o, double x_, double y_) {
    int x0_ = std::lower_bound(Unique_x[o].begin(), Unique_x[o].end(), x_) - Unique_x[o].begin();
    if(fabs(Unique_x[o][x0_] - x_) < eps || x0_ == 0); else x0_--;
    int y0_ = std::lower_bound(Unique_y[o].begin(), Unique_y[o].end(), y_) - Unique_y[o].begin();
    if(fabs(Unique_y[o][y0_] - y_) < eps || y0_ == 0); else y0_--;
    return map_id[x0_][y0_];
}//获得x,y对应的离散化id

void WayFinding2::Insert_Edge(int o, int u, int v, double dis) {
    Head[o][u] = Edge_Num[o];
    Edge_[o][Edge_Num[o]++] = (Edge){u, v, Head[o][u], dis};
}

void WayFinding2::Init() {
    memset(Head, -1, sizeof(Head));
    memset(map_id, -1, sizeof(map_id));
    int cnt_robot = 0, cnt_workbench = 0;
    for (int i = 0; i < map_size_; i++)
        for (int j = 0; j < map_size_; j++) {
            double px = j * 0.5 + 0.25;
            double py = (map_size_ - i - 1) * 0.5 + 0.25;
            for(int o = 0; o < 2; o++) {//两种图判一下建边
                for(int k = 0; k < 9; k++) {
                    double dx = direction_x[o][k];
                    double dy = direction_y[o][k];
                    double nx = px + dx, ny = py + dy;
//                    if(nx < eps || nx > 50 - eps || ny < eps || ny > 50 - eps) continue;
                    //无脑离散化， 只要不建边就是不合法的点
                    Unique_x[o].push_back(nx);
                    Unique_y[o].push_back(ny);
                }
            }
        }
    Unique(Unique_x[0]);
    Unique(Unique_x[1]);
    Unique(Unique_y[0]);
    Unique(Unique_y[1]);
    //初始化建图， 先将所有的点离散化之后再开始建边

    for(int i = 0; i < map_size_; ++i) {
        for(int j = 0; j < map_size_; ++j) {
            if(map_[i][j] == '#') {
                double px = j * 0.5 + 0.25;
                joint_obs_x0_.push_back(px + 0.25);
                joint_obs_x0_.push_back(px - 0.25);
            }
        }
    }

    /*
     * 对所有的障碍点集进行去重
     */
    Unique(joint_obs_x0_);
    for (int i = 0; i < map_size_; i++) {
        for (int j = 0; j < map_size_; j++) if(map_[i][j] == '#'){
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

    for(int o = 0; o < 2; ++o) {
        for (int s = 0; s < (int) workbench.size(); ++s) {
            Dijkstra(o, s);
            for (int j = 1; j <= 14; ++j) {
                for (int i = 0; i < N_; ++i) {
                    From[o][s][i][j] = From[o][s][From[o][s][i][j - 1]][j - 1];
                }
            }
        }
    }

}

std::priority_queue<Status> Q;
void WayFinding2::Dijkstra(int o, int s) { //有没有拿东西o:0/1, 起点： 工作台
    for(int i = 0; i < N_; ++i) {
        From[o][s][i][0] = -1;
        Dis[o][s][i] = INF;
    }
    Dis[o][s][s] = 0.0;
    while(!Q.empty()) Q.pop();
    Q.push(Status{s, Dis[o][s][s]});
    while (!Q.empty()) {
        auto x = Q.top(); Q.pop();
        int u = x.u;
        if (Dis[o][s][u] != x.d) continue;
        for (int k = Head[o][u]; k != -1; k = Edge_[o][k].next) {
            // Log::print("o: ", o, "s: ", s, "u: ", u, "k: ", k);
            if (Dis[o][s][Edge_[o][k].to] > Dis[o][s][Edge_[o][k].from] + Edge_[o][k].dis) {
                Dis[o][s][Edge_[o][k].to] = Dis[o][s][Edge_[o][k].from] + Edge_[o][k].dis;
                From[o][s][Edge_[o][k].to][0] = Edge_[o][k].from; // 前驱点
                Q.push(Status{Edge_[o][k].to, Dis[o][s][Edge_[o][k].to]});
            }
        }
    }
}