#include "wayfinding.h"
#include "geometry.h"
#include <algorithm>
#include <queue>

using namespace Geometry;

std::vector<double> dijk_d_;
std::vector<Geometry::Point> joint_walk_, joint_obs_, workbench_pos;
std::vector<int> edges_;
std::vector<std::vector<Route>> routes_;

WayFindding::WayFindding(char (*map)[map_size_]): map_(map) {
    for (int i = 0; i < map_size_; i++) for (int j = 0; j < map_size_; j++) {
        if ('1' <= map[i][j] && map[i][j] <= '9') {
            workbench_pos.push_back({i * 0.5 + 0.25, j * 0.5 + 0.25});
        }
        if (map[i][j] != '#') continue;
        for (auto [di, dj] : {{1, -1},{1, 1},{-1, -1},{-1, 1}}) {
            int ni = di+i, nj = dj+j;
            if (ni < 0 || ni >= map_size_ || nj < 0 || nj >= map_size_) continue;
            if (!(map[ni][nj] != '#' || map[ni][j] != '#' || map[i][nj] != '#')) continue;
            double pi = i * 0.5 + 0.25 + di * 0.25;
            double pj = j * 0.5 + 0.25 + dj * 0.25;
            joint_obs_.push_back({pi, pj});
            pi += di * 0.5;
            pj += dj * 0.5;
            joint_walk_.push_back({pi, pj});
        }
    }
    for (auto& uni : {joint_walk_, joint_obs_}) {
        std::sort(begin(uni), end(uni));
        uni.resize(std::unique(begin(uni), end(uni)) - begin(uni));
    }
    for (size_t i = 0; i < joint_walk_.size() + workbench_pos.size(); i++) for (size_t j = 0; j < i; j++) {
        auto& a = GetGraphPoint(i);
        auto& b = GetGraphPoint(j);
        bool valid = true;
        for (auto& k : joint_obs_) {
            double d = DistanceToSegment(k, a, b);
            if (d < 0.5 - 1e-3) {
                valid = false;
                break;
            }
        }
        if (!valid) continue;
        // 暂不考虑单行道
        edges_[i].push_back(j);
        edges_[j].push_back(i);
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

Point WayFindding::GetGraphPoint(int i) {
    return i < joint_walk_.size() ? joint_walk_[i] : workbench_pos[i - joint_walk_.size()];
}

double WayFindding::DistBetweenPoints(Point a, Point b) {
    return Geometry::Dist(a, b) + 1e-3; // 走直线则只走端点
}
void WayFindding::Dijkstra(int s) {
    // TODO：将方向放入状态
    dijk_d_.resize(joint_.size());
    static const double INF = 1e18;
    for (auto& i : dijk_d_) i = INF;
    std::priority_queue<Status> Q;
    dijk_d_[s] = 0.0;
    Q.push({s, dijk_d_[s]});
    while (Q.size()) {
        auto [u, du] = Q.top(); Q.pop();
        if (dijk_d_[u] != du) continue;
        for (auto v : edges_[u]) {
            double d = distBetweenPoints(GetGraphPoint(u), GetGraphPoint(v)); 
            if (dijk_d_[v] > dijk_d_[u] + d) {
                dijk_d_[v] = dijk_d_[u] + d;
                Q.push({v, dijk_d_[v]});
            }
        }
    }
}


