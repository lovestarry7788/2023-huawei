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

    constexpr int map_size_ = Input::map_size_;

    struct Edge {
        int u, v;
        double dist;
        double dist_wall; // 离墙4端点最近距离
    };

    extern size_t dijk_siz_;
    extern std::vector<double> dijk_d_;
    extern std::vector<int> dijk_p_;
    extern std::vector<std::vector<int>> E_;
    extern std::vector<Edge> edges_;

    extern std::vector<Geometry::Point> joint_walk_, joint_obs_, workbench_pos;
    extern std::vector<std::vector<Route>> routes_;

    int FreeSpace(int x, int y, int dx, int dy, int mx);
    void Init();
    Point GetGraphPoint(int i);
    double DistBetweenPoints(Point a, Point b);
    void Dijkstra(int s);
    struct Status  {
        int p;
        // int orient; // 离散化
        double d;
        
        bool operator<(const Status& rhs) const {
            return d > rhs.d;
        }
    };

};

#endif