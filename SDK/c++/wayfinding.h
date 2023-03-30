#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"

using Route = std::vector<Geometry::Point>;
struct WayFindding {
    char (*map_)[100];
    std::vector<double> dijk_d_;
    std::vector<Geometry::Point> joint_;
    std::vector<Route> routes_;

    WayFindding(char (*map)[100]);
    void dijkstra(int s);
    void calc_all_route();
};

#endif