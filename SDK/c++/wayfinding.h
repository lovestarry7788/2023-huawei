#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>

namespace WayFinding {
    using Route = std::vector<Geometry::Point>;
    static constexpr double INF = 1e18;

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
    extern std::vector<std::vector<double> > dist[2];
    extern std::vector<std::vector<int> > pre[2];
    extern std::vector<Edge> edge[2];
    extern std::vector<int> head[2];

    extern std::vector<Geometry::Point> joint_walk_, joint_obs_, workbench_pos, robot_pos;
    extern std::vector<std::vector<Route>> routes_[2];

    int FreeSpace(int x, int y, int dx, int dy, int mx);
    void Insert_Edge(int o, int u, int v, double dis, double dis_to_wall);
    void Init();
    // void Init_Frame();
    Geometry::Point GetGraphPoint(int i);
    double DistBetweenPoints(Geometry::Point a, Geometry::Point b);
    double CalcDistance(int id, int workbench_i, int workbench_j);
    double CalcFrame(int id, int i, int j);
    void Dijkstra(int o, int s, bool(*valid)(int t) = nullptr);
    bool GetOfflineRoute(int o, Geometry::Point cnt, int workbench_id, Route& output);
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
    double DistToWall(Geometry::Point p, double ori);
};

#endif