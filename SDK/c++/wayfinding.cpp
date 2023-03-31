#include "wayfinding.h"
#include "geometry.h"
#include "input.h"
#include "log.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>

using namespace Geometry;
using namespace Input;
using namespace WayFinding;

size_t WayFinding::dijk_siz_;
std::vector<double> WayFinding::dijk_d_;
std::vector<int> WayFinding::dijk_p_;
std::vector<std::vector<int>> WayFinding::E_;
std::vector<Edge> WayFinding::edges_;

std::vector<Geometry::Point> WayFinding::joint_walk_, WayFinding::joint_obs_, WayFinding::workbench_pos;
std::vector<std::vector<Route>> WayFinding::routes_;

int WayFinding::FreeSpace(int x, int y, int dx, int dy, int mx) {
    int ans = 0;
    bool valid = true;
    bool P = x == 45 && y ==2;
    for (int ta = 1; ta <= mx && valid; ta++) {
        valid = 0 <= x+ta*dx && x+ta*dx < map_size_ && 0 <= y+ta*dy && y+ta*dy < map_size_ &&
            map_[x+ta*dx][y+ta*dy] != '#';
        for (int x2 = 0; x2 < ta && valid; x2++) valid &= map_[x+x2*dx][y+ta*dy] != '#';
        for (int y2 = 0; y2 < ta && valid; y2++) valid &= map_[x+ta*dx][y+y2*dy] != '#';
        if (valid) ans = ta;
    }
    return ans;
}
void WayFinding::Init() {
    double r[2] = {0.47, 0.53};
    for (int i = 0; i < map_size_; i++) for (int j = 0; j < map_size_; j++) {
        double px = j * 0.5 + 0.25;
        double py = (map_size_ - i - 1) * 0.5 + 0.25;
        if ('1' <= map_[i][j] && map_[i][j] <= '9') {
            workbench_pos.push_back({px, py});
        }
        if (map_[i][j] != '#') continue;
        joint_obs_.push_back({px + 0.25, py + 0.25}); 
        joint_obs_.push_back({px + 0.25, py - 0.25}); 
        joint_obs_.push_back({px - 0.25, py + 0.25}); 
        joint_obs_.push_back({px - 0.25, py - 0.25}); 
        // std::vector<std::pair<int,int>> dij;
        for (auto [di, dj] : std::vector<std::pair<int,int>>{{1, -1},{1, 1},{-1, -1},{-1, 1}}) {
            int ni = di+i, nj = dj+j;
            // if (ni == 29 && nj == 93) Log::print("tt", i, j, map_[ni][nj], map_[ni][j], map_[i][nj]);
            if (ni < 0 || ni >= map_size_ || nj < 0 || nj >= map_size_) continue;
            if (!(map_[ni][nj] != '#' && map_[ni][j] != '#' && map_[i][nj] != '#')) continue;
            int minsp = FreeSpace(i, j, di, dj, 3);
            if (minsp < 2) continue;
            // Log::print("angle", ni, nj, px, py, minsp);

            // 每个90度点只生成一个路径点，视两侧最小宽度，为2则移动一格，大于等于3则移动1.5格。由于最短路，如果不想进入2格宽的路，就不会经过移动1格生成的点而撞。
            double px2 = px + dj * 0.25 * (minsp + 1);
            double py2 = py + -di * 0.25 * (minsp + 1);
            joint_walk_.push_back({px2, py2});
        }
    }
    auto unique = [](std::vector<Point> &uni) {
        std::sort(begin(uni), end(uni));
        auto p = std::unique(begin(uni), end(uni));
        uni.resize(p - begin(uni));
    };
    unique(joint_obs_);
    unique(joint_walk_);
    // for (auto i : joint_walk_) {
    //     Log::print("joint_walk_", i.x, i.y);
    // }
    dijk_siz_ = joint_walk_.size() + workbench_pos.size();
    E_.resize(dijk_siz_);
    for (int i = 0; i < dijk_siz_; i++) for (int j = 0; j < i; j++) {
        auto a = GetGraphPoint(i);
        auto b = GetGraphPoint(j);
        bool valid = true;
        double mind = 1e18;
        for (auto& k : joint_obs_) {
            double d = DistanceToSegment(k, a, b);
            mind = std::min(mind, d);
            // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
            if (d < 0.45 + 2e-2) { // 操作误差
                valid = false;
                break;
            }
            if (!valid) continue;
            // 暂不考虑单行道
            edges_[k][i].push_back(j);
            edges_[k][j].push_back(i);
        }
        // Log::print("dijk", i, j);
        if (!valid) continue;
        double d = DistBetweenPoints(a, b);
        // 暂不考虑单行道，存了mind供将来判断道路宽度使用
        E_[i].push_back(edges_.size()), edges_.push_back({i, j, d, mind});
        E_[j].push_back(edges_.size()), edges_.push_back({j, i, d, mind});
    }
    int workbench_num = workbench_pos.size();
    routes_.resize(workbench_num, std::vector<Route>(workbench_num));
    for (int i = 0; i < workbench_num; i++) {
        int s = i + joint_walk_.size();
        Dijkstra(s);
        for (int j = 0; j < workbench_num; j++) {
            int t = j + joint_walk_.size();
            auto& route = routes_[i][j];
            route.clear();
            while (t != s) {
                route.push_back(GetGraphPoint(t));
                t = edges_[dijk_p_[t]].u;
            }
        }
    }
}

Point WayFinding::GetGraphPoint(int i) { // 函数内部
    return i < joint_walk_.size() ? joint_walk_[i] : workbench_pos[i - joint_walk_.size()];
}

double WayFinding::DistBetweenPoints(Point a, Point b) {
    return Geometry::Length(a - b) + 1e-3; // 走直线则只走端点
}

void WayFinding::Dijkstra(int s) {
    // TODO：将方向放入状态
    static const double INF = 1e18;
    dijk_p_.assign(dijk_siz_, -1);
    dijk_d_.assign(dijk_siz_, INF);
    std::priority_queue<Status> Q;
    dijk_d_[s] = 0.0;
    Q.push({s, dijk_d_[s]});
    while (!Q.empty()) {
        auto [u, du] = Q.top(); Q.pop();
        if (dijk_d_[u] != du) continue;
        for (auto ei : E_[u]) {
            auto [_, v, d, __] = edges_[ei];
            if (dijk_d_[v] > dijk_d_[u] + d) {
                dijk_d_[v] = dijk_d_[u] + d;
                dijk_p_[v] = ei;
                Q.push({v, dijk_d_[v]});
            }
        }
    }
}