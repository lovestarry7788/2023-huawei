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

size_t WayFinding::N;
std::vector<double> WayFinding::dist;
std::vector<int> WayFinding::pre;
std::vector<Edge> WayFinding::edge;
std::vector<int> WayFinding::head;

std::vector<Geometry::Point> WayFinding::joint_walk_, WayFinding::joint_obs_, WayFinding::workbench_pos, WayFinding::robot_pos;
std::vector<std::vector<Route> > WayFinding::routes_;

void WayFinding::Insert_Edge(int u, int v, int dis, int dis_to_wall) {
    edge.push_back(Edge{u, v, head[u], dis, dis_to_wall});
    int edge_num = edge.size() - 1;
    head[u] = edge_num;
}

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
//    double Radius[2] = {0.47, 0.53};
    for (int i = 0; i < map_size_; i++) {
        for (int j = 0; j < map_size_; j++) {
            double px = j * 0.5 + 0.25;
            double py = (map_size_ - i - 1) * 0.5 + 0.25;

            if (map_[i][j] == 'A') {
                robot_pos.push_back({px, py});
            } else if ('1' <= map_[i][j] && map_[i][j] <= '9') {
                workbench_pos.push_back({px, py});
            }

            if (map_[i][j] != '#') continue;
            joint_obs_.push_back({px + 0.25, py + 0.25});
            joint_obs_.push_back({px + 0.25, py - 0.25});
            joint_obs_.push_back({px - 0.25, py + 0.25});
            joint_obs_.push_back({px - 0.25, py - 0.25});
            // std::vector<std::pair<int,int>> dij;
            for (const auto &[di, dj]: std::vector<std::pair<int, int> >{{1,  -1},
                                                                         {1,  1},
                                                                         {-1, -1},
                                                                         {-1, 1}}) {
                int ni = di + i, nj = dj + j;
                if (ni < 0 || ni >= map_size_ || nj < 0 || nj >= map_size_) continue;
                if (!(map_[ni][nj] != '#' && map_[ni][j] != '#' && map_[i][nj] != '#')) continue;
                int minsp = FreeSpace(i, j, di, dj, 3); // 自动计算过道大小.
                if (minsp < 2) continue;
                // Log::print("angle", ni, nj, px, py, minsp);

                // 每个90度点只生成一个路径点，视两侧最小宽度，为2则移动一格，大于等于3则移动1.5格。由于最短路，如果不想进入2格宽的路，就不会经过移动1格生成的点而撞。
                double px2 = px + dj * 0.25 * (minsp + 1);
                double py2 = py + -di * 0.25 * (minsp + 1);
                joint_walk_.push_back({px2, py2});
            }
        }
    }

    /*
     * 对所有的点集进行去重
     */
    auto unique = [](std::vector<Point> &uni) {
        std::sort(begin(uni), end(uni));
        auto p = std::unique(begin(uni), end(uni));
        uni.resize(p - begin(uni));
    };

    unique(joint_obs_);
    unique(joint_walk_);

    /*
     * 离散化的点包括：机器人初始位置 + 工作台 + 障碍的拐角所拓展的点
     * 对于所有离散化的点进行通过测试。
     */
    N = robot_pos.size() + workbench_pos.size() + joint_walk_.size();
    head.assign(N, -1); // 清空邻接表
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < i; j++) {
            auto a = GetGraphPoint(i); // 编号： 先是可以走的点，再是工作台
            auto b = GetGraphPoint(j);
            bool valid = true;
            double mind = 1e18;
            for (const auto& k: joint_obs_) {
                double d = DistanceToSegment(k, a, b);
                mind = std::min(mind, d);
                // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
                if (d < 0.45 + 2e-2) { // 操作误差
                    valid = false;
                    break;
                }
            }
            if (!valid) continue;
            double d = DistBetweenPoints(a, b);
            // 暂不考虑单行道，存了mind供将来判断道路宽度使用
            Insert_Edge(i, j, d, mind);
            Insert_Edge(j, i, d, mind);
        }
    }

    /*
     * M 表示 机器人 + 工作台 的数量
     * 规划 M * M 的走法
     * route_[i][j] 表示从 i 到 j 的路径集合。
     */
    int M = robot_pos.size() + workbench_pos.size();
    Log::print(N, M);
    routes_.resize(M, std::vector<Route>(M));
    for (int s = 0; s < M; s++) {
        Dijkstra(s);
        for (int j = 0; j < M; j++) if(dist[j] < INF) {
            int t = j;
            auto& route = routes_[s][t];
            route.clear();
            // 从终点一直添加路径到起点
            Log::print(s, t);
            while (t != s) {
                route.push_back(GetGraphPoint(t));
                t = edge[pre[t]].u;
            }
            std::reverse(route.begin(), route.end());
        }
    }

    Log::print("WayFinding Ready!");
}

Point WayFinding::GetGraphPoint(int i) { // 函数内部
    if(i < robot_pos.size()) return robot_pos[i];
    else if(i < workbench_pos.size()) return workbench_pos[i - robot_pos.size()];
    return joint_walk_[i - robot_pos.size() - workbench_pos.size()];
}

double WayFinding::DistBetweenPoints(Point a, Point b) {
    return Geometry::Length(a - b) + 1e-3; // 走直线则只走端点
}

void WayFinding::Dijkstra(int s) {
    // TODO：将方向放入状态
    pre.assign(N, -1);
    dist.assign(N, INF);
    std::priority_queue<std::pair<double, int> > Q;
    dist[s] = 0.0;
    Q.push({dist[s], s});
    while (!Q.empty()) {
        auto x = Q.top(); Q.pop();
        if (dist[x.second] != x.first) continue;
        for (int k = head[x.second]; k != -1; k = edge[k].nex) {
            if (dist[edge[k].v] > dist[edge[k].u] + edge[k].dis) {
                dist[edge[k].v] = dist[edge[k].u] + edge[k].dis;
                pre[edge[k].v] = k;
                Q.push({dist[edge[k].v], edge[k].v});
            }
        }
    }
}