#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include "input.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>

using Route = std::vector<Geometry::Point>;
namespace WayFinding {
    using namespace Input;
    using namespace Geometry;
    constexpr double INF = 1e18;
    constexpr int map_size_ = Input::map_size_;

    struct Edge {
        int u, v, nex;
        double dis; // 距离
        double dis_to_wall; // 离墙4端点最近距离
        Edge(){}
        Edge(int _u, int _v, int _nex, int _dis, int _dis_to_wall) : u(_u), v(_v), nex(_nex), dis(_dis), dis_to_wall(_dis_to_wall){}
    };

    extern size_t N;
    extern std::vector<double> dist;
    extern std::vector<int> pre;
    extern std::vector<Edge> edge;
    extern std::vector<int> head;

    extern std::vector<Geometry::Point> joint_walk_, joint_obs_, workbench_pos, robot_pos;
    extern std::vector<std::vector<Route>> routes_;

    int FreeSpace(int x, int y, int dx, int dy, int mx);
    void Insert_Edge(int u, int v, int dis, int dis_to_wall);
    void Init();
    Point GetGraphPoint(int i);
    double DistBetweenPoints(Point a, Point b);
    void Dijkstra(int s);
};

#endif