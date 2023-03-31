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
    extern std::vector<double> dijk_d_;
    extern std::vector<Point> joint_walk_[2], joint_obs_, workbench_pos;
    extern std::vector<int> edges_[2][10001];
    extern std::vector<std::vector<Route>> routes_;

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