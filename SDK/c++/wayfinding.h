#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>
#include <array>

namespace WayFinding2 {
    using Route = std::vector<Geometry::Point>;
    static constexpr double INF = 1e18;
    static constexpr int K_ = 9;//一个格子拆分成多少个小格子
    static constexpr int N_xy = 500;
    static constexpr int N_ = 11000;
    static constexpr int M_ = N_ * 20;//8联通， 双向
//    static constexpr int M_ = N_ * 648;

    struct Edge {
        int from, to, next;
        double dis; // 距离
        Edge(){}
        Edge(int _u, int _v, int _nex, double _dis)
            : from(_u), to(_v), next(_nex), dis(_dis){}
    };

    struct Status {
        int u;
        double d;
        Status(int u, double d) : u(u), d(d) {}
        bool operator<(const Status& rhs) const {
            return d > rhs.d;
        }
    };

    const double Radius[2] = {0.45, 0.53};
    extern int Edge_Num[2];//建边， 边的数量
    extern Edge Edge_[2][M_];
    extern int Head[2][N_];//建边, 领接表头指针
    extern Geometry::Point Point_[2][60000];//编号对应的点的坐标
    extern std::vector<double> Unique_x;//用于离散化x坐标
    extern std::vector<double> Unique_y;//用于离散化y坐标
    extern std::vector<double> joint_obs_x0_, joint_obs_[101];
    extern std::map<std::array<int,2>, int> map_id;

    template<typename T> void Unique(std::vector<T> &a);//离散化a
    int GetID(int o, double x_, double y_);//获得x,y对应的离散化id
    void Insert_Edge(int o, int u, int v, double dis);

    const int dx[8] = {
            0, 0, 1, -1, 1, -1, 1, -1
    };
    const int dy[8] = {
            1, -1,  0, 0, 1, -1, -1, 1
    };

    const double direction_x[2][9] = {
            //上下左右0.4没用， 碰不到够不到
            //右下角可以选{0.28, -0.27}, 点买完之后会再被往下挤0.23左右， 也就是去到0.5
//            {0},
//            {0}
            {0, 0.28, 0.28, -0.28, -0.28, -0.27, 0.27, -0.27, 0.27},
            {0, 0.28, 0.28, -0.28, -0.28, -0.5, 0.5, -0.5, 0.5}
    };
    const double direction_y[2][9] = {
            //对应的方位， 机器人买完之后， 会被挤到哪里
//            {0},
//            {0}
            {0, -0.27, 0.27, -0.27, 0.27, 0.28, 0.28, -0.28, -0.28},
            {0, -0.5, 0.5, -0.5, 0.5, 0.28, 0.28, -0.28, -0.28}
    };

    extern double Dis[2][400][N_];//最短路： 初始建图的最短路
//    extern double Dis2[2][400][N_];//最短路2：根据FindRoute更新最短路情况

    const int Binary_Limit = 12;
    extern int From[2][400][N_][Binary_Limit];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点
    void Init();//拆解地图， 每一个格子都拆成K_个点， K_往4个方向联通
    void Dijkstra(int o, int s, int &total_time);//有没有拿东西o:0/1, 起点： 工作台
    bool Check_Avoid_Crash_Wall(int o, int A, int B);//测试沿着AB连线走会不会撞墙
    int FindRoute(int o, int A, int B);//倍增， 找一个AB路径中离A最远的P， 使得A能直线到P
    std::array<int,2> GetBlockID(double x0_, double y0_);
    extern std::vector<std::array<double,2>> PointInBlock[2][100][100];
};

#endif
