#include "wayfinding.h"
#include "geometry.h"
#include "input.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>

using namespace Geometry;
using namespace Input;

std::vector<double> dijk_d_;
std::vector<Geometry::Point> joint_walk_[2], joint_obs_, workbench_pos;
std::vector<int> edges_[2][10001];
std::vector<std::vector<Route> > routes_;

void WayFinding::Init() {
    double r[2] = {0.47, 0.53};
    for (int i = 0; i < map_size_; i++) for (int j = 0; j < map_size_; j++) {
        if ('1' <= map_[i][j] && map_[i][j] <= '9') {
            workbench_pos.push_back({i * 0.5 + 0.25, j * 0.5 + 0.25});
        }
        if (map_[i][j] != '#') continue;
        for (const auto& [di, dj] : {{1, -1},{1, 1},{-1, -1},{-1, 1}}) {
            int ni = di+i, nj = dj+j;
            if (ni < 0 || ni >= map_size_ || nj < 0 || nj >= map_size_) continue;
            if (!(map_[ni][nj] != '#' || map_[ni][j] != '#' || map_[i][nj] != '#')) continue;
            double pi = i * 0.5 + 0.25 + di * 0.25;
            double pj = j * 0.5 + 0.25 + dj * 0.25;
            joint_obs_.push_back({pi, pj});
            pi += di * 0.5;
            pj += dj * 0.5;
            joint_walk_[0].emplace_back({pi, pj});
            pi += di * 0.25;
            pj += dj * 0.25;
            joint_walk_[1].emplace_back({pi, pj});
        }
    }
    for (auto& uni : {joint_walk_, joint_obs_}) {
        std::sort(begin(uni), end(uni));
        uni.resize(std::unique(begin(uni), end(uni)) - begin(uni));
    }

    for(int k = 0; k < 2; ++k) {
        for (size_t i = 0; i < joint_walk_.size() + workbench_pos.size(); i++) for (size_t j = 0; j < i; j++) {
            auto a = GetGraphPoint(i);
            auto b = GetGraphPoint(j);
            bool valid = true;
            for (const auto& u : joint_obs_) {
                double d = DistanceToSegment(u, a, b);
                if (d <= r[k] + 1e-3) {
                    valid = false;
                    break;
                }
            }
            if (!valid) continue;
            // 暂不考虑单行道
            edges_[k][i].push_back(j);
            edges_[k][j].push_back(i);
        }
    }

    size_t workbench_num = workbench_pos.size();
    routes_.resize(workbench_num, std::vector<Route>(workbench_num));
    for (size_t i = 0; i < workbench_pos.size(); i++) {
        int s = i + joint_walk_.size();
        Dijkstra(s);
        for (size_t j = 0; j < workbench_pos.size(); j++) {
            int t = j + joint_walk_.size();
            auto& route = routes_[i][j];
            route.clear();
            while (t != s) {
                route.push_back(t);
                // dijk add lst
            }
        }

    }
}

Point WayFinding::GetGraphPoint(int i) {
    return i < joint_walk_.size() ? joint_walk_[i] : workbench_pos[i - joint_walk_.size()];
}

double WayFinding::DistBetweenPoints(Point a, Point b) {
    return Geometry::Dist(a, b) + 1e-3; // 走直线则只走端点
}

void WayFinding::Dijkstra(int s) {
    // TODO：将方向放入状态
    dijk_d_.resize(joint_.size());
    static const double INF = 1e18;
    for (auto& i : dijk_d_) i = INF;
    std::priority_queue<Status> Q;
    dijk_d_[s] = 0.0;
    Q.push({s, dijk_d_[s]});
    while (!Q.empty()) {
        auto [u, du] = Q.top(); Q.pop();
        if (dijk_d_[u] != du) continue;
        for (const auto& v: edges_[u]) {
            double d = DistBetweenPoints(GetGraphPoint(u), GetGraphPoint(v));
            if (dijk_d_[v] > dijk_d_[u] + d) {
                dijk_d_[v] = dijk_d_[u] + d;
                Q.push({v, dijk_d_[v]});
            }
        }
    }
}