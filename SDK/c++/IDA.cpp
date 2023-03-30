//
// Created by 刘智杰 on 2023/3/29.
//

#include "input.h"
#include "IDA.h"
#include "robot.h"
#include "workbench.h"
#include <utility>
#include <cmath>
#include <vector>

int frameID, coins, K;
char map_[101][101];
std::vector<std::shared_ptr<Workbench> > workbench;
std::vector<std::shared_ptr<Robot> > robot;
constexpr int robot_num_ = 4;

void IDA::AStar(double sx, double sy, double dx, double dy, std::vector<Geometry::Point> planned_route) {

}

bool IDA::Have_Obstacle(double sx, double sy, double dx, double dy) {
    auto s = InWhichCell(sx, sy);
    auto d = InWhichCell(dx, dy);
    bool have_obstacle = false;
    for(int i = std::min(s.first, d.first); i <= std::max(s.first, d.first); ++i) {
        for(int j = std::min(s.second, d.second); j <= std::max(s.second, d.second); ++j) {
            std::vector<Geometry::Point> v = {{i * 0.5, j * 0.5}, {(i+1) * 0.5, j * 0.5}, {i * 0.5, (j+1)*0.5}, {(i+1) * 0.5, (j+1) * 0.5}};
            have_obstacle |= Geometry::CheckCross({sx, sy}, {dx, dy}, v);
        }
    }
    return have_obstacle;
}


bool IDA::Connection(double sx, double sy, double dx, double dy) {
    auto s = InWhichCell(sx, sy);
    auto d = InWhichCell(dx, dy);
    return static_cast<bool>(gDis[s.first][s.second][d.first][d.second] < static_cast<int>(1e9));
}

double IDA::gCalcTime(double sx, double sy, double dx, double dy) {
    auto s = InWhichCell(sx, sy);
    auto d = InWhichCell(dx, dy);
    return gDis[s.first][s.second][d.first][d.second];
}

std::pair<int,int> IDA::InWhichCell(double sx, double sy) {
    return std::pair<int,int>{static_cast<int>(sx/0.5) , static_cast<int>(sy/0.5)};
}

void IDA::Bfs(int sx, int sy) {
    std::queue<std::pair<int,int> > q;
    int dx[9] = {-1, -1, 0, 1, 1, 1, 0, -1};
    int dy[9] = {0, 1, 1, 1, 0, -1, -1, -1};
    gDis[sx][sy][sx][sy] = 0;

    while(!q.empty()) {
        std::pair<int,int> x = q.front(); q.pop();
        for(int k = 0; k < 8; ++k) {
            std::pair<int,int> y = {x.first + dx[k], x.second + dy[k]};
            if(y.first >= 0 && y.second >= 0 && y.first < 100 && y.second < 100) {
                if(gDis[sx][sy][y.first][y.second] > static_cast<int>(1e9)) q.push({y.first, y.second});
                gDis[sx][sy][y.first][y.second] = std::min(gDis[sx][sy][y.first][y.second], gDis[sx][sy][x.first][x.second] + 1);
            }
        }
    }
}

void IDA::Init() {
    memset(gDis, 63, sizeof gDis);
    for(int sx = 0; sx < 100; ++sx) {
        for(int sy = 0; sy < 100; ++sy) {
            if(Input::is_obstacle_[sx][sy] == false) {
                Bfs(sx, sy);
            }
        }
    }

    for(int idx = 0; idx < Input::robot_num_ + Input::K; ++idx) {
        for(int idy = 0; idy < Input::robot_num_ + Input::K; ++idy) {
            double sx, sy, dx, dy;
            if(idx < 4) {
                sx = robot[idx] -> x0_;
                sy = robot[idx] -> y0_;
            } else {
                sx = workbench[idx - robot_num_] -> x0_;
                sy = workbench[idx - robot_num_] -> y0_;
            }
            if(idy < 4) {
                dx = robot[idy] -> x0_;
                dy = robot[idy] -> y0_;
            } else {
                dx = workbench[idy - robot_num_] -> x0_;
                dy = workbench[idy - robot_num_] -> y0_;
            }

            std::vector<Geometry::Point> planned_route;
            AStar(sx, sy, dx, dy, planned_route);
            swap(planned_route_[idx][idy], planned_route);
        }
    }
}