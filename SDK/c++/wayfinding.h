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
        Edge(int _u, int _v, int _nex, double _dis, double _dis_to_wall) : u(_u), v(_v), nex(_nex), dis(_dis), dis_to_wall(_dis_to_wall){}
    };

    extern double Dis[2][50][N_];//最短路： 初始建图的最短路
    extern double Dis2[2][50][N_];//最短路2：
    extern int From[2][50][N_][15];//状态， 工作台编号， 终点， 2^k步： 从终点出发到对应的工作台的最短路上， 走2^k到达哪个点
    void Init();//拆解地图， 每一个格子都拆成K_个点， K_往4个方向联通
    void Dijkstra(int o, int s);//有没有拿东西o:0/1, 起点： 工作台
    int FindRoute(int o, int A, int B);//倍增， 找一个AB路径中离A最远的P， 使得A能直线到P
    bool Check_Avoid_Crash_Wall(int o, int A, int B);//测试沿着AB连线走会不会撞墙


};

#endif