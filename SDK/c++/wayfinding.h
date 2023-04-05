#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>
#include <array>

namespace WayFinding {
    using Route = std::vector<Geometry::Point>;
    static constexpr double INF = 1e18;
    static constexpr int N_ = 2001;

    struct Edge {
        int u, v, nex;
        double dis; // 距离
        double dis_to_wall; // 离墙4端点最近距离
        Edge(){}
        Edge(int _u, int _v, int _nex, double _dis, double _dis_to_wall) : u(_u), v(_v), nex(_nex), dis(_dis), dis_to_wall(_dis_to_wall){}
    };

    struct Status {
        int u;
        double d;
        Status(int u, double d) : u(u), d(d) {}
        bool operator<(const Status& rhs) const {
            return d > rhs.d;
        }
    };

    extern size_t N;
    extern double dist[2][N_][N_];
    extern int pre[2][N_][N_];
    extern Edge edge[2][N_ * 20];
    extern int len[2];
    extern int head[2][N_];

    extern std::vector<Geometry::Point> joint_walk_, workbench_pos, robot_pos;
    extern Route routes_[2][N_][N_];
    extern std::vector<double> joint_obs_[101];

    int FreeSpace(int x, int y, int dx, int dy, int mx);
    void Insert_Edge(int o, int u, int v, double dis, double dis_to_wall);
    void Init();
    // void Init_Frame();
    Geometry::Point GetGraphPoint(int i);
    double DistBetweenPoints(Geometry::Point a, Geometry::Point b);
    double CalcDistance(int id, int workbench_i, int workbench_i_direction, int workbench_j, int workbench_j_direction);
    double CalcFrame(int id, int i, int j);
    void Dijkstra(int o, int s, bool(*valid)(int t) = nullptr);
    bool GetOfflineRoute(int o, Geometry::Point cnt,
                                     int from,
                                     int workbench_to_id, int workbench_to_direction,
                                     Route& output);
    Route GetOnlineRoute(int o, int s, int t);

    template<typename T> void Unique(T& Uni);

    const std::vector<std::vector<std::array<int,2>>> Wall = {
            {{-1, -1}, {-2, -1}, {-1, -2}, {-2, -2}, {-3, -1}, {-1, -3}, {-3, -2}, {-2, -3}},
            {{1, -1}, {2, -1}, {1, -2}, {2, -2}, {3, -1}, {1, -3}, {3, -2}, {2, -3}},
            {{1, 1}, {2, 1}, {1, 2}, {2, 2}, {3, 1}, {1, 3}, {3, 2}, {2, 3}},
            {{-1, 1}, {-2, 1}, {-1, 2}, {-2, 2}, {-3, 1}, {-1, 3}, {-3, 2}, {-2, 3}}
    };
    const Geometry::Point direction[] = {
            {0.5, 0}, {0, 0}, {0, -0.5}, {0.5, -0.5}
    };
    const std::vector<std::vector<std::array<int,2>>> row = {
            {{-1, 0}, {-2, 0}, {-3, 0}},
            {{0, -1}, {0, -2}, {0, -3}},
            {{0, 1}, {0, 2}, {0, 3}},
            {{1, 0}, {2, 0}, {3, 0}}
    };
    const std::vector<std::array<double, 2>> workbench_extern = {
//            {-0.27, 0.27},  {0, 0.27},  {0.27, 0.27},
//            {-0.27, 0},     {0, 0},          {0.27, 0},
//            {-0.27, -0.27},  {0, -0.27}, {0.27, -0.27}

//            {-0.27, 0.27},  {0, 0.38},  {0.27, 0.27},
//            {-0.38, 0},                     {0.38, 0},
//            {-0.27, -0.27},  {0, -0.38}, {0.27, -0.27}//, {0.5, -0.25},
//            {-0.25, -0.5},                            {0.25, -0.5},

//            {0.1249, 0.3747}, {0.2793, 0.2793}, {0.3747, 0.1249},
//            {-0.1249, 0.3747}, {-0.2793, 0.2793}, {-0.3747, 0.1249},
//            {0.1249, -0.3747}, {0.2793, -0.2793}, {0.3747, -0.1249},
//            {-0.1249, -0.3747}, {-0.2793, -0.2793}, {-0.3747, -0.1249}

            {0.1225, 0.3674}, {0.2739, 0.2739}, {0.3674, 0.1225},
            {-0.1225, 0.3674}, {-0.2739, 0.2739}, {-0.3674, 0.1225},
            {0.1225, -0.3674}, {0.2739, -0.2739}, {0.3674, -0.1225},
            {-0.1225, -0.3674}, {-0.2739, -0.2739}, {-0.3674, -0.1225}

//            {0, 0}
    };//工作台的扩展, 这些点可以买到货
//    const std::vector<std::array<double, 2>> walking_extern = {
//            {-0.5, 0.5}, {-0.25, 0.5}, {0, 0.5}, {0.25, 0.5}, {0.5, 0.5},
//            {-0.5, 0.25},                                     {0.5, 0.25},
//            {-0.5, 0},                                        {0.5, 0},
//            {-0.5, -0.25},                                    {0.5, -0.25},
//            {-0.5, -0.5}, {-0.25, -0.5}, {0, -0.5}, {0.25, 0.5}, {0.5, -0.5}
//    };//防止被挤出来之后找不着北
    extern int workbench_extern_id[50][12];
    double DistToWall(Geometry::Point p, double ori);
};

#endif