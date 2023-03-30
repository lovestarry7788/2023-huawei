#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include "input.h"

using Route = std::vector<Geometry::Point>;
namespace WayFindding {
    constexpr int map_size_ = Input::map_size_;
    extern std::vector<double> dijk_d_;
    extern std::vector<Geometry::Point> joint_walk_, joint_obs_, workbench_pos;
    extern std::vector<int> edges_;
    extern std::vector<std::vector<Route>> routes_;

    void dijkstra(int s);

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