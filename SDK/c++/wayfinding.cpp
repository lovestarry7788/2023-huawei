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

int WayFinding2::FindRoute(int o, int A, int B) {
    return -1;//表示没有找到P点
}
void WayFinding2::Unique(std::vector<double> &a);//离散化a
int WayFinding2::GetID(double x_, double y_);//获得x,y对应的离散化id
void WayFinding2::Insert_Edge(int o, int u, int v, double dis);
void WayFinding2::Init();//拆解地图， 每一个格子都拆成K_个点， K_往4个方向联通
void WayFinding2::Dijkstra(int o, int s);//有没有拿东西o:0/1, 起点： 工作台
bool WayFinding2::Check_Avoid_Crash_Wall(int o, int A, int B);//测试沿着AB连线走会不会撞墙