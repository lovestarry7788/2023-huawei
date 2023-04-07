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
double WayFinding2::Dis[2][50][N_];//最短路： 初始建图的最短路
double WayFinding2::Dis2[2][50][N_];//最短路2：
int WayFinding2::From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点


bool WayFinding2::Check_Avoid_Crash_Wall(int o, int A, int B){
    return true;
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
//            if(map_[i][j] == 'A')
        }
}
void WayFinding2::Dijkstra(int o, int s);//有没有拿东西o:0/1, 起点： 工作台