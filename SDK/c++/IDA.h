//
// Created by 刘智杰 on 2023/3/29.
//

#ifndef CODECRAFTSDK_IDA_H
#define CODECRAFTSDK_IDA_H

#include "geometry.h"
#include "workbench.h"
#include "robot.h"
#include "input.h"
#include <vector>
#include <iostream>
#include <utility>

namespace IDA {

    int gDis[101][101][101][101];
    extern std::vector<Geometry::Point> planned_route_[110][110]; // 前四是机器人，后面是工作台，存的是工作路过的点。

    void AStar(double sx, double sy, double dx, double dy, std::vector<Geometry::Point> planned_route);
    bool Have_Obstacle(double sx, double sy, double dx, double dy);
    bool Connection(double sx, double sy, double dx, double dy);
    double gCalcTime(double sx, double sy, double dx, double dy);
    std::pair<int,int> InWhichCell(double sx, double sy);
    void Bfs(int sx, int sy);
    void Init();
}

#endif //CODECRAFTSDK_IDA_H
