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
    static constexpr int N_ = 50 * 50 * K_;

    struct Edge {
        int u, v, nex;
        double dis; // 距离
        double dis_to_wall; // 离墙4端点最近距离
        Edge(){}
        Edge(int _u, int _v, int _nex, double _dis) : u(_u), v(_v), nex(_nex), dis(_dis){}
    };

    extern int cnt;//建边， 边的数量
    extern int Head[2][N_];//建边, 领接表头指针
    extern Geometry::Point Point_[2][N_];//编号对应的点的坐标
    extern std::vector<double> Unique_x[2];//用于离散化x坐标
    extern std::vector<double> Unique_y[2];//用于离散化y坐标

//    int cnt;//建边， 边的数量
//    int Head[2][N_];//建边, 领接表头指针
//    Geometry::Point Point_[2][N_];//编号对应的点的坐标
//    std::vector<double> Unique_x[2];//用于离散化x坐标
//    std::vector<double> Unique_y[2];//用于离散化y坐标
//    double Dis[2][50][N_];//最短路： 初始建图的最短路
//    double Dis2[2][50][N_];//最短路2：
//    int From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点

    void Unique(std::vector<double> &a);//离散化a
    int GetID(double x_, double y_);//获得x,y对应的离散化id
    void Insert_Edge(int o, int u, int v, double dis);

    const double direction_x[2][9] = {
            //上下左右0.4没用， 碰不到够不到
            //右下角可以选{0.28, -0.27}, 点买完之后会再被往下挤0.23左右， 也就是去到0.5
            {0, 0.28, 0.28, -0.28, -0.28, 0.25, 0.25, -0.25, -0.25},
            {0, 0.28, 0.28, -0.28, -0.28, 0.25, 0.25, -0.25, -0.25}
    };
    const double direction_y[2][9] = {
            //对应的方位， 机器人买完之后， 会被挤到哪里
            {0, -0.27, 0.27, -0.27, 0.27, 0.25, -0.25, 0.25, -0.25},
            {0, -0.5, 0.5, -0.5, 0.5, 0.25, -0.25, 0.25, -0.25}

    };

    extern double Radius[2] = {0.45, 0.53};
    extern double Dis[2][50][N_];//最短路： 初始建图的最短路
    extern double Dis2[2][50][N_];//最短路2：
    extern int From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点
    extern std::vector<double> joint_obs_x0_;
    extern std::vector<double> joint_obs_[101];
    extern double dist[2][N_][N_];
    extern int pre[2][N_][N_];
    extern Edge edge[2][N_ * 20];
    extern int len[2];
    extern int head[2][N_];

    void Init();//拆解地图， 每一个格子都拆成K_个点， K_往4个方向联通
    void Insert_Edge(int o, int u, int v, double dis);
    void Dijkstra(int o, int s);//有没有拿东西o:0/1, 起点： 工作台
    int FindRoute(int o, int A, int B);//倍增， 找一个AB路径中离A最远的P， 使得A能直线到P
    bool Check_Avoid_Crash_Wall(int o, int A, int B);//测试沿着AB连线走会不会撞墙


};

#endif