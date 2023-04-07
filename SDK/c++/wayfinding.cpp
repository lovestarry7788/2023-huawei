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

int WayFinding2::cnt;//建边， 边的数量
int WayFinding2::Head[2][N_];//建边, 领接表头指针
Geometry::Point WayFinding2::Point_[2][N_];//编号对应的点的坐标
std::vector<double> WayFinding2::Unique_x[2];//用于离散化x坐标
std::vector<double> WayFinding2::Unique_y[2];//用于离散化y坐标
double WayFinding2::Dis[2][50][N_];//最短路： 初始建图的最短路
double WayFinding2::Dis2[2][50][N_];//最短路2：
int WayFinding2::From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点
std::vector<double> WayFinding2::joint_obs_x0_;
std::vector<double> WayFinding2::joint_obs_[101];

int WayFinding2::FindRoute(int o, int A, int B) {
    return -1;//表示没有找到P点
}
void WayFinding2::Unique(std::vector<double> &a);//离散化a
int WayFinding2::GetID(double x_, double y_);//获得x,y对应的离散化id
void WayFinding2::Insert_Edge(int o, int u, int v, double dis) {
    len[o] ++;
    edge[o][len[o]] = Edge{u, v, head[o][u], dis};
    head[o][u] = len[o];
}
void WayFinding2::Init() { //拆解地图， 每一个格子都拆成K_个点， K_往4个方向联通

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

}
void WayFinding2::Dijkstra(int o, int s) { //有没有拿东西o:0/1, 起点： 工作台

}

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
}