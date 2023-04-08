#ifndef HW2023_WAYFINDING_H
#define HW2023_WAYFINDING_H

#include "geometry.h"
#include "input.h"
#include "log.h"
#include <algorithm>
#include <queue>
#include <vector>
#include <utility>
#include <iostream>
#include <array>

namespace WayFinding {
    using Route = std::vector<Geometry::Point>;
    using namespace Geometry;
    using namespace Input;

    template<typename T>
    void Unique(T& uni) {
        std::sort(begin(uni), end(uni));
        auto p = std::unique(begin(uni), end(uni));
        uni.resize(p - begin(uni));
    }

    struct Edge {
        int u, v;
        double dis; // 距离
        // double dis_to_wall; // 离墙4端点最近距离
        Edge(int _u, int _v, double _dis) : u(_u), v(_v), dis(_dis){}
    };

    struct Status {
        int u;
        double d;
        Status(int u, double d) : u(u), d(d) {}
        bool operator<(const Status& rhs) const {
            return d > rhs.d;
        }
    };
    // extern int test[Input::map_size_][map_size_]; // 先每个点中心离散化

    struct WayFinding {
        static constexpr double INF = 1e18;
        const double radius = 0.53;
        // const int map_size_ = 100;
        size_t N1, N2;

        std::vector<Point> random_pos;
        int random_pos_to_id[map_size_][map_size_]; // 先每个点中心离散化
        std::vector<int> workbench_to_gid;

        std::vector<Point> joint_obs;

        std::vector<std::vector<double>> dist1; // n1 to n1
        std::vector<std::vector<double>> dist2; // n2 to n1
        std::vector<std::vector<int>> nxt2, order2; // n2 to n1

        std::vector<Point> vertex;
        std::vector<Edge> edges;
        std::vector<std::vector<int>> E; 

        // wb在图中的id
        // int get_workbench_id(int wb_id) {
        //     return ;
        // }
        // 随机点在此序列中的id
        int get_random_id(Point cnt) {
            int pi = int((50 - cnt.y) / 0.5), pj = int(cnt.x / 0.5);
            return random_pos_to_id[pi][pj];
        }
        Point nxt_point(Point cnt, int graph_id) {
            int random_id = get_random_id(cnt);
            Log::print("nxt_point", random_id, nxt2[random_id][graph_id]);
            return vertex[nxt2[random_id][graph_id]];
        }
        double DistBetweenPoints(Point a, Point b) {
            return Length(a - b) + 1e-1;
        }
        void init() {
            std::vector<Point> workbench_pos;
            // random_pos.resize(map_size_, std::vector<Point>(map_size_));
            for (int i = 0; i < map_size_; i++) {
                for (int j = 0; j < map_size_; j++) {
                    double px = j * 0.5 + 0.25;
                    double py = (map_size_ - i - 1) * 0.5 + 0.25;
                    random_pos.push_back(Point{px, py});
                    if (map_[i][j] == 'A') 
                        continue;
                    if ('1' <= map_[i][j] && map_[i][j] <= '9') {
                        workbench_pos.push_back({px, py});
                        vertex.push_back({px, py});
                        continue;
                    }
                    if (map_[i][j] != '#') continue;
                    for (const auto &[di, dj]: std::vector<std::pair<int, int> >{{1,  -1},
                                                                                {1,  1},
                                                                                {-1, -1},
                                                                                {-1, 1}}) {
                        joint_obs.push_back({px + dj * 0.25, py - di * 0.25});

                        int ni = di + i, nj = dj + j;
                        if (ni < 0 || ni >= map_size_ || nj < 0 || nj >= map_size_) continue;
                        if (!(map_[ni][nj] != '#' && map_[ni][j] != '#' && map_[i][nj] != '#')) continue;
                        int minsp = FreeSpace(i, j, di, dj, 3); // 自动计算过道大小.
                        if (minsp < 2) continue;

                        double px2 = px + dj * (0.25 * 1 + radius + 1e-4);
                        double py2 = py + -di * (0.25 * 1 + radius + 1e-4);
                        vertex.push_back({px2, py2});
                        px2 = px + dj * (0.25 * 1 + radius / sqrt(2) + 1e-4);
                        py2 = py + -di * (0.25 * 1 + radius / sqrt(2) + 1e-4);
                        vertex.push_back({px2, py2});
                        // Log::print("angleadd", i, j, di, dj);
                    }
                }
            }
            Unique(vertex);
            Unique(joint_obs);
            // Unique(random_pos);
            N1 = vertex.size();
            N2 = random_pos.size();

            for (auto i : workbench_pos)  
                workbench_to_gid.push_back(lower_bound(begin(vertex), end(vertex), i) - begin(vertex));
            for (int i = 0; i < map_size_; i++) {
                for (int j = 0; j < map_size_; j++) {
                    random_pos_to_id[i][j] = i * map_size_ + j;
                }
            }
            Log::print(131);
            // build edges and E
            E.resize(N1);
            for (int i = 0; i < N1; i++) {
                for (int j = 0; j < i; j++) {
                    if (IsCollideRouteToWall(vertex[i], vertex[j])) {
                        continue;
                    }
                    double d = DistBetweenPoints(vertex[i], vertex[j]);
                    E[i].push_back(edges.size());
                    edges.push_back(Edge{i, j, d});
                    E[j].push_back(edges.size());
                    edges.push_back(Edge{j, i, d});
                }
            }
            Log::print("N is", N1, N2, joint_obs.size(), edges.size());

            Log::print(146);

            // calc dist1
            dist1.resize(N1);
            for (size_t i = 0; i < N1; i++) {
                dist1[i].resize(N1, INF);
                Dijkstra(i, dist1[i]);
            }
            Log::print(154);
            
            // calc dist2 and nxt2
            dist2.resize(N2);
            nxt2.resize(N2);
            order2.resize(N2);
            for (size_t i = 0; i < N2; i++) {
                if (__builtin_popcount(i)== 1) Log::print(i);
                dist2[i].resize(N1, INF);
                nxt2[i].resize(N1, -1);
                for (size_t j = 0; j < N1; j++)
                    order2[i].push_back(j);
                std::sort(begin(order2[i]), end(order2[i]), [&](int l, int r) {
                    return DistBetweenPoints(random_pos[i], vertex[l]) < 
                            DistBetweenPoints(random_pos[i], vertex[r]);
                });
                // std::priority_queue<Status> Q;
                auto& dist = dist2[i];
                for (size_t j = 0; j < N1; j++) {
                    // if (IsCollideRouteToWall(random_pos[i], vertex[j])) continue;
                    double d = DistBetweenPoints(random_pos[i], vertex[j]);
                    dist[j] = d;
                    // Q.push({int(j), d});
                    // Log::print(random_pos[i], vertex[j]);
                    // if (dist2[i][j] <= d) continue;
                    // dist2[i][j] = d;
                    // nxt2[i][j] = j;
                    for (int wb : workbench_to_gid) {
                        if (dist[wb] > dist[j] + dist1[j][wb]) {
                            dist[wb] = dist[j] + dist1[j][wb];
                            nxt2[i][wb] = j;
                        }
                    }
                    // for (size_t k = 0; k < N1; k++) {
                    // // for (int ei : E[j]) {
                    // //     int k = edges[ei].v;
                    //     if (dist2[i][k] > dist2[i][j] + dist1[j][k]) {
                    //         dist2[i][k] = dist2[i][j] + dist1[j][k];
                    //         nxt2[i][k] = j;
                    //     }
                    // }
                }
                // while (Q.size()) {
                //     auto [u, du] = Q.top(); Q.pop();
                //     if (dist[u] != du) continue;
                //     for (int ei : E[u]) {
                //         auto [_, v, d] = edges[ei];
                //         if (dist[v] > dist[u] + d) {
                //             dist[v] = dist[u] + d;
                //             Q.push({v, dist[v]});
                //         }
                //     }
                // }
                std::sort(begin(order2[i]), end(order2[i]), [&](int l, int r) {
                    return dist[l] < dist[r];
                });
                while (order2[i].size() && dist[order2[i].back()] == INF)
                    order2[i].pop_back();
            }
        }
        int FreeSpace(int x, int y, int dx, int dy, int mx) {
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
        bool IsCollideRouteToWall(Point a, Point b) {
            for (auto obs : joint_obs) {
                if (DistanceToSegment(obs, a, b) < radius + 1e-4)
                    return true;
            }
            return false;
        }
        void Dijkstra(int s, std::vector<double> &dist) {
            std::priority_queue<Status> Q;
            dist[s] = 0.0;
            Q.push(Status{s, dist[s]});
            while (!Q.empty()) {
                auto x = Q.top(); Q.pop();
                int u = x.u;
                if (dist[u] != x.d) continue;
                for (int ei : E[u]) {
                    auto [_, v, d] = edges[ei];
                    if (dist[v] > dist[u] + d) {
                        dist[v] = dist[u] + d;
                        Q.push({v, dist[v]});
                    }
                }
            }
        }
    };

    extern WayFinding Way;


    // const std::vector<std::vector<std::array<int,2>>> Wall = {
    //         {{-1, -1}, {-2, -1}, {-1, -2}, {-2, -2}, {-3, -1}, {-1, -3}, {-3, -2}, {-2, -3}},
    //         {{1, -1}, {2, -1}, {1, -2}, {2, -2}, {3, -1}, {1, -3}, {3, -2}, {2, -3}},
    //         {{1, 1}, {2, 1}, {1, 2}, {2, 2}, {3, 1}, {1, 3}, {3, 2}, {2, 3}},
    //         {{-1, 1}, {-2, 1}, {-1, 2}, {-2, 2}, {-3, 1}, {-1, 3}, {-3, 2}, {-2, 3}}
    // };
    // const Geometry::Point direction[] = {
    //         {0.5, 0}, {0, 0}, {0, -0.5}, {0.5, -0.5}
    // };
    // const std::vector<std::vector<std::array<int,2>>> row = {
    //         {{-1, 0}, {-2, 0}, {-3, 0}},
    //         {{0, -1}, {0, -2}, {0, -3}},
    //         {{0, 1}, {0, 2}, {0, 3}},
    //         {{1, 0}, {2, 0}, {3, 0}}
    // };
    // const std::vector<std::array<double, 2>> workbench_extern = {
//            {-0.27, 0.27},  {0, 0.27},  {0.27, 0.27},
//            {-0.27, 0},     {0, 0},          {0.27, 0},
//            {-0.27, -0.27},  {0, -0.27}, {0.27, -0.27}

//            {-0.27, 0.27},  {0, 0.38},  {0.27, 0.27},
//            {-0.38, 0},                     {0.38, 0},
//            {-0.27, -0.27},  {0, -0.38}, {0.27, -0.27}//, {0.5, -0.25},
//            {-0.25, -0.5},                            {0.25, -0.5},

//            {0.1249, 0.3747}, {0.2793, 0.2793}, {0.3747, 0.1249},
//            {-0.1249, 0.3747}, {-0.2793, 0.2793}, {-0.3747, 0.1249},
//            {0.1249, -0.3747}, {0.2793, -0.2793}, {0.3747, -0.1249},
//            {-0.1249, -0.3747}, {-0.2793, -0.2793}, {-0.3747, -0.1249}

            // {0.1225, 0.3674}, {0.2739, 0.2739}, {0.3674, 0.1225},
            // {-0.1225, 0.3674}, {-0.2739, 0.2739}, {-0.3674, 0.1225},
            // {0.1225, -0.3674}, {0.2739, -0.2739}, {0.3674, -0.1225},
            // {-0.1225, -0.3674}, {-0.2739, -0.2739}, {-0.3674, -0.1225}

//            {0, 0}
    // };//工作台的扩展, 这些点可以买到货
//    const std::vector<std::array<double, 2>> walking_extern = {
//            {-0.5, 0.5}, {-0.25, 0.5}, {0, 0.5}, {0.25, 0.5}, {0.5, 0.5},
//            {-0.5, 0.25},                                     {0.5, 0.25},
//            {-0.5, 0},                                        {0.5, 0},
//            {-0.5, -0.25},                                    {0.5, -0.25},
//            {-0.5, -0.5}, {-0.25, -0.5}, {0, -0.5}, {0.25, 0.5}, {0.5, -0.5}
//    };//防止被挤出来之后找不着北
    // extern int workbench_extern_id[50][12];
    // double DistToWall(Geometry::Point p, double ori);
};

#endif