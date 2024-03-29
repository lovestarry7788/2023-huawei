#include "wayfinding.h"
#include "geometry.h"
#include "input.h"
#include "log.h"
#include <algorithm>
#include <queue>
#include <map>
#include <vector>
#include <utility>
#include <iostream>

using namespace Geometry;
using namespace Input;
using namespace WayFinding;

size_t WayFinding::N;
//int Input::map_id_[map_size_][map_size_];
//拆点后不能用map_id_逆映射回去， 考虑使用set + lower_bound
//std::map<std::array<int,2>, int> robot_id_;//为了防止浮点数误差， 对坐标*100后存入映射
//std::map<std::array<int,2>, int> workbench_id_;
std::map<std::array<int,2>, int> map_id_;
double WayFinding::dist[2][N_][N_];
int WayFinding::pre[2][N_][N_];
Edge WayFinding::edge[2][N_ * 20];
int WayFinding::head[2][N_];
int WayFinding::len[2];
int WayFinding::workbench_extern_id[50][12];
std::pair<int,int> WayFinding::dis_mn_[1210][101][101]; // 存的是 pair<direction, direction>
double WayFinding::mn_[1210][101][101];

std::vector<Geometry::Point> WayFinding::joint_walk_, WayFinding::workbench_pos, WayFinding::robot_pos;
std::vector<double> WayFinding::joint_obs_[101];
Route WayFinding::routes_[2][N_][N_];

void WayFinding::Insert_Edge(int o, int u, int v, double dis, double dis_to_wall) {
    len[o] ++;
    edge[o][len[o]] = Edge{u, v, head[o][u], dis, dis_to_wall};
    head[o][u] = len[o];
}

template<typename T>
void WayFinding::Unique(T& uni) {
        std::sort(begin(uni), end(uni));
        auto p = std::unique(begin(uni), end(uni));
        uni.resize(p - begin(uni));
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
    std::vector<double> joint_obs_x0_, joint_obs_y0_;
    double Radius[2] = {0.45, 0.53}; len[0] = len[1] = 0;
//    memset(map_id_, -1, sizeof(map_id_));
//    robot_id_.clear();
//    workbench_id_.clear();
    map_id_.clear();
    int cnt_robot = 0, cnt_workbench = 0;
    for (int i = 0; i < map_size_; i++) {
        for (int j = 0; j < map_size_; j++) {
            double px = j * 0.5 + 0.25;
            double py = (map_size_ - i - 1) * 0.5 + 0.25;
            if (map_[i][j] == 'A') {
                robot_pos.push_back({px, py});
            } else if ('1' <= map_[i][j] && map_[i][j] <= '9') {
//                workbench_pos.push_back({px, py});
                for(int k = 0; k < workbench_extern.size(); k++) {//周围的点
                    auto [dx, dy] = workbench_extern[k];
                    double nx = px + dx, ny = py + dy;
                    if(nx < eps || nx > 50 - eps || ny < eps || ny > 50 - eps) {
                        workbench_extern_id[cnt_workbench][k] = -1;
                        continue;
                        //如果会越出去那就没这个点的存在， 需要特别判掉
                    } else {
                        workbench_extern_id[cnt_workbench][k] = workbench_pos.size();
                        workbench_pos.push_back({nx, ny});
//                        Log::print("workbench_id: ", cnt_workbench, " workbench_id_direciton: ", workbench_pos.size() + robot_num_, " ", k, " ", nx, " ", ny, '\n');
                        //记录一下可以到的点对应的id
                    }
                }
                cnt_workbench++;
                //只能表示这个位置有人用。
            }

            if (map_[i][j] != '#') continue;
            joint_obs_x0_.push_back(px + 0.25);
            joint_obs_x0_.push_back(px - 0.25);

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
                double pad = std::min(2.5, minsp + 0.0);
                double px2 = px + dj * 0.25 * (1 + pad);
                double py2 = py + -di * 0.25 * (1 + pad);
                joint_walk_.push_back({px2, py2});
            }
        }
    }

    /*
     * 对所有的点集进行去重
     */
    Unique(joint_obs_x0_);
    Unique(joint_walk_);
    // Log::print("x0_size: ", joint_obs_x0_.size());

    for (int i = 0; i < map_size_; i++) {
        for (int j = 0; j < map_size_; j++) if(map_[i][j] == '#'){
            double px = j * 0.5 + 0.25;
            double py = (map_size_ - i - 1) * 0.5 + 0.25;
            int idx;
            idx = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), px + 0.25) - joint_obs_x0_.begin();
            joint_obs_[idx].push_back(py + 0.25);
            joint_obs_[idx].push_back(py - 0.25);

            idx = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), px - 0.25) - joint_obs_x0_.begin();
            joint_obs_[idx].push_back(py + 0.25);
            joint_obs_[idx].push_back(py - 0.25);
        }
    }

    for(int i = 0; i < joint_obs_x0_.size(); ++i) {
        Unique(joint_obs_[i]);
    }

    /*
     * 离散化的点包括：机器人初始位置 + 工作台 + 障碍的拐角所拓展的点
     * 对于所有离散化的点进行通过测试。
     */
    N = robot_pos.size() + workbench_pos.size() + joint_walk_.size();
    Log::print("N: ", N);
    /*
    Log::print("N: ", N, "joint_obs_: ", joint_obs_.size());
    for(const auto& obs_: joint_obs_) {
        Log::print("x: ", obs_.x, "y: ", obs_.y);
    }
    */
//    for(int i = 0; i < N; ++i) {
//        auto a = GetGraphPoint(i);
//        Log::print("a.x: ", a.x, "a.y: ", a.y);
//    }


    double x0, x1, y0, y1;
    int x0_, x1_, y0_, y1_;
    double d;

    for (int o = 0; o < 2; ++o) {
        for (int i = 0; i < N; ++i) head[o][i] = -1;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < i; j++) {
                auto a = GetGraphPoint(i); // 编号： 先是可以走的点，再是工作台
                auto b = GetGraphPoint(j);
                bool valid = true;
                double mind = 1e18;
                x0 = std::min(a.x, b.x) - Radius[o], x1 = std::max(a.x, b.x) + Radius[o], y0 = std::min(a.y, b.y) - Radius[o], y1 = std::max(a.y, b.y) + Radius[o];
                x0_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x0) - joint_obs_x0_.begin() - 1; x0_ = std::max(x0_, 0);
                x1_ = std::lower_bound(joint_obs_x0_.begin(), joint_obs_x0_.end(), x1) - joint_obs_x0_.begin();
                for (int x = x0_; x <= x1_; ++x) if(!joint_obs_[x].empty()){
                    y0_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y0) - joint_obs_[x].begin() - 1; y0_ = std::max(y0_, 0);
                    y1_ = std::lower_bound(joint_obs_[x].begin(), joint_obs_[x].end(), y1) - joint_obs_[x].begin();
                    for (int y = y0_; y <= y1_; ++y) {
                        d = DistanceToSegment({joint_obs_x0_[x], joint_obs_[x][y]}, a, b);
                        mind = std::min(mind, d);
                        // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
                        if (d < Radius[o] + 2e-2) { // 操作误差
                            valid = false;
                            break;
                        }
                    }
                    if(!valid) break;
                }
//                 Log::print(i, j, a.x, a.y, b.x, b.y, d, mind, valid);
                // 暂不考虑单行道，存了mind供将来判断道路宽度使用
                if (valid) {
                    d = DistBetweenPoints(a, b);
                    Insert_Edge(o, i, j, d, mind);
                    Insert_Edge(o, j, i, d, mind);
                }
            }
        }
    }
//    Log::print("Insert_Edge Done!");

    /*
     * M 表示 机器人 + 工作台 的数量
     * 规划 M * M 的走法
     * route_[i][j] 表示从 i 到 j 的路径集合。
     */
    int M = robot_pos.size() + workbench_pos.size();
    Log::print("N: ", N, "M: ", M, "len[0]: ", len[0], "len[1]: ", len[1]);

    for(int o = 0; o < 2; ++o) {
        for (int s = 0; s < M; s++) {
            Dijkstra(o, s);
            // Log::print("o: ", o, "s: ", s);
             Log::print(s, GetGraphPoint(s).x, GetGraphPoint(s).y);
            for (int t = 0; t < M; t++)
                if (dist[o][s][t] < INF) {
                    routes_[o][s][t] = GetOnlineRoute(o, s, t);
                }
        }
    }

//    Log::print("Dijkstra Done");
    /*
     * 预处理两点间最近的 direction
     */
    int workbench_size = WayFinding::workbench_pos.size();
    for(int id_s = 0; id_s <= robot_num_ + workbench_size; ++id_s)
        for(int i = 0; i < K; ++i)
            for(int j = 0; j < K; ++j) mn_[id_s][i][j] = 1e9;

    for(int id_s = 0; id_s <= robot_num_ + WayFinding::workbench_pos.size(); ++id_s) {
        for (int i = 0; i < K; ++i) {
            for (int i_direction = 0; i_direction < WayFinding::workbench_extern.size(); i_direction++) {
                if (WayFinding::workbench_extern_id[i][i_direction] != -1) {
                    int id_i = robot_num_ + WayFinding::workbench_extern_id[i][i_direction];
                    for (int j = 0; j < K; ++j) {
                        for (int j_direction = 0; j_direction < WayFinding::workbench_extern.size(); j_direction++) {
                            if (WayFinding::workbench_extern_id[j][j_direction] != -1) {
                                int id_j = robot_num_ + WayFinding::workbench_extern_id[j][j_direction];
                                double buy_sell_frame_ = WayFinding::dist[0][id_s][id_i] + WayFinding::dist[1][id_i][id_j];
                                if(mn_[id_s][i][j] > buy_sell_frame_) {
                                    mn_[id_s][i][j] = buy_sell_frame_;
                                    dis_mn_[id_s][i][j] = std::make_pair(i_direction, j_direction);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Log::print("WayFinding Ready!");
}

/*
 * 对于每一帧，更新机器人的状态。
void WayFinding::Init_Frame() {
    double Radius[2] = {0.47, 0.53};
    N = robot_pos.size() + workbench_pos.size() + joint_walk_.size();
    for (int o = 0; o < 2; ++o) {
        for (int i = 0; i < robot_num_; i++) {
            head[o][i] = -1;
            for (int j = robot_num_; j < N; j++) {
                auto a = GetGraphPoint(i); // 编号： 先是可以走的点，再是工作台
                auto b = GetGraphPoint(j);
                bool valid = true;
                double mind = 1e18;
                for (const auto &k: joint_obs_) {
                    double d = DistanceToSegment(k, a, b);
                    mind = std::min(mind, d);
                    // 细小问题，两点间直线，只有一个瓶颈，中间有空的，仍可以通过多辆车。no，没问题，将防碰撞提前处理了部分。
                    if (d < Radius[o] + 2e-2) { // 操作误差
                        valid = false;
                        break;
                    }
                }
                double d = DistBetweenPoints(a, b);
                // Log::print(i, j, a.x, a.y, b.x, b.y, d, mind, valid);
                // 暂不考虑单行道，存了mind供将来判断道路宽度使用
                if (valid) {
                    Insert_Edge(o, i, j, d, mind);
                    Insert_Edge(o, j, i, d, mind);
                }
            }
        }
    }
    for (int o = 0; o < 2; ++o) {
        for (int s = 0; s < robot_num_; s++) {
            Dijkstra(o, s);
        }
    }
}
*/

Point WayFinding::GetGraphPoint(int i) { // 函数内部
    if(i < robot_pos.size()) return robot_pos[i];
    else if(i < workbench_pos.size() + robot_pos.size()) return workbench_pos[i - robot_pos.size()];
    return joint_walk_[i - robot_pos.size() - workbench_pos.size()];
}

double WayFinding::DistBetweenPoints(Point a, Point b) {
    return Geometry::Length(a - b) + 1e-3; // 走直线则只走端点
}

Route WayFinding::GetOnlineRoute(int o, int s, int t) {
    //有没有装东西， 起点， 终点
//    int s = workbench_extern_id[workbench_id][workbench_id_direction];
    Route route;
    while (t != s) {
        route.push_back(GetGraphPoint(t));
        t = edge[o][pre[o][s][t]].u;
    }
    route.push_back(GetGraphPoint(s)); // 用于回退后到路径
    std::reverse(route.begin(), route.end());
    return route;
}

std::priority_queue<Status> Q;
void WayFinding::Dijkstra(int o, int s, bool(*valid)(int t)) {
    // TODO：将方向放入状态
    for(int i = 0; i < N; ++i) {
        pre[o][s][i] = -1;
        dist[o][s][i] = INF;
    }
    dist[o][s][s] = 0.0;
    while(!Q.empty()) Q.pop();
    Q.push(Status{s, dist[o][s][s]});
    while (!Q.empty()) {
        auto x = Q.top(); Q.pop();
        int u = x.u;
        if (dist[o][s][u] != x.d) continue;
        if (valid && valid(u)) return;
        for (int k = head[o][u]; k != -1; k = edge[o][k].nex) {
            // Log::print("o: ", o, "s: ", s, "u: ", u, "k: ", k);
            if (dist[o][s][edge[o][k].v] > dist[o][s][edge[o][k].u] + edge[o][k].dis) {
                dist[o][s][edge[o][k].v] = dist[o][s][edge[o][k].u] + edge[o][k].dis;
                pre[o][s][edge[o][k].v] = k;
                Q.push(Status{edge[o][k].v, dist[o][s][edge[o][k].v]});
            }
        }
    }
}

// 实时找躲避路径，但不实时找到工作台路径。
bool WayFinding::GetOfflineRoute(int o, Point cnt,
                                 int from,
                                 int workbench_to_id, int workbench_to_direction,
                                 Route& output) {
//    int pi = int((50 - cnt.y) / 0.5), pj = int(cnt.x / 0.5);
//    int from = workbench_extern_id[workbench_from_id][workbench_from_direction];

    //map_id_[pi][pj];
//    if(map_id_.find({int((cnt.x + 0.125) * 4), int((cnt.y + 0.125) * 4)}) != map_id_.end())
//        from = map_id_[{int((cnt.x + 0.125) * 4), int((cnt.y + 0.125) * 4)}];
//    else
//        from = -1;
    //from获取对应的位置， 枚举direction可以获得
    Log::print("from: ", cnt.x, " ", cnt.y, " ", from, '\n');
    if (from == -1)
        return false;

    int to = workbench_extern_id[workbench_to_id][workbench_to_direction] + robot_num_;

    output = routes_[o][from][to];
    Log::print("frame: ", frameID, "from: ", from, "to: ", to, "from.x: ", GetGraphPoint(from).x, "from.y: ", GetGraphPoint(from).y, "to.x: ", GetGraphPoint(to).x, "to.y: ", GetGraphPoint(to).y);
    Log::print(workbench_to_id, " ", workbench_to_direction, " ", output.size(), '\n');
    /*
        Log::print("frame: ", frameID, "from: ", from, "to: ", workbench_id + robot_pos.size(), "from.x: ", GetGraphPoint(from).x, "from.y: ", GetGraphPoint(from).y, "to.x: ", GetGraphPoint(workbench_id + robot_pos.size()).x, "to.y: ", GetGraphPoint(workbench_id + robot_pos.size()).y);
        for(const auto& u: output) {
            Log::print(u.x, u.y);
        }
    */
    return output.size() > 0;
}

/*
 * 计算机器人 id -> workbench[i] -> workbench[j] 的距离。
 */
double WayFinding::CalcDistance(int id, int workbench_i, int workbench_i_direciton, int workbench_j, int workbench_j_direction) {
    int pi = int((50 - robot[id] -> pos_.y) / 0.5), pj = int(robot[id] -> pos_.x / 0.5);
    Log::print("Frame: ", frameID, "CalcDistance, id: ", id, "last_point_: ", robot[id] -> last_point_);
    int id_i = robot_num_ + workbench_extern_id[workbench_i][workbench_i_direciton];
    int id_j = robot_num_ + workbench_extern_id[workbench_j][workbench_j_direction];
    return dist[0][robot[id] -> last_point_][id_i] + dist[1][id_i][id_j];
}

/*
 * 计算机器人 id -> workbench[i] -> workbench[j] 的帧数
 */
double WayFinding::CalcFrame(int id, int i, int j) {
    return 1e9;
}

double WayFinding::DistToWall(Point p, double orient) {
    double mind = 100;
    Vector ori{cos(orient), sin(orient)};
    const static std::vector<std::pair<Point, double>> wall{
            {{0,0.40}, 0},
            {{49.60,0}, PI/2},
            {{50,49.60}, PI},
            {{0.40,50}, -PI/2},
    };
    for (const auto& [wp, wo] : wall) {
        Point sec = GetLineIntersection2(p, ori, wp, {cos(wo), sin(wo)});
        // if (Input::frame)
        if (Dot(sec - p, ori) > 0) {
            mind = std::min(mind, Length(sec - p));
            // if (Input::frameID == 1069 && id_ == 0) {
            //     Log::print(id_, Length(sec - p), ori.x, ori.y);
            //     Log::print(p.x, p.y);
            // }
        }
    }
    return mind;
}

//double WayFinding::DistToWall(Geometry::Point p, double ori) {
//    int position_i = p.x / 0.5, position_j = 100 - ceil(p.y / 0.5);
//    //获得所在方块的坐标
//    double ans = 2;
//    //先根据所在象限判断， 然后再根据
////    using Geometry::PI;
//    double PI = Geometry::PI;
//    double eps = Geometry::eps;
//
//    auto work = [&](int id) {
//        for(auto p2: Wall[id]) {
//            int i = position_i + p2[0], j = position_j + p2[1];
//            if(i < 0 || i > 99 || j < 0 || j > 99) continue;
//            if(Input::is_obstacle_[i][j]) {
//                Point Wall_ = {i * 0.5 + direction[id].x, 50 - j * 0.5 + direction[id].y};
//                ans = fmin(ans, Dist(p, Wall_));
//                break;
//            }
//        }
//    };
//
//    auto work_in_row = [&](int id) {
//        for(auto p2: row[id]) {
//            int i = position_i + p2[0], j = position_j + p2[1];
//            if(i < 0 || i > 99 || j < 0 || j > 99) continue;
//            if(Input::is_obstacle_[i][j]) {
//                Point Wall_ = {i * 0.5 + direction[id].x, 50 - j * 0.5 + direction[id].y};
//                ans = fmin(ans, fabs(Wall_.x - p.x));
//                break;
//            }
//        }
//    };
//
//    if(ori + PI/2 < eps && ori + PI > -eps) { // 4
//        work(0);
//    }
//    if(ori < -eps && ori + PI / 2 > eps) { // 3
//        work(1);
//    }
//    if(ori - PI/2 < eps && ori > -eps) { // 2
//        work(2);
//    }
//    if(ori - PI / 2 > -eps && ori - PI < eps) { // 1
//        work(3);
//    }
//
//    if(fabs(ori - PI) < eps || fabs(ori + PI) < eps) {
//        work(0); work(4); work_in_row(0);
//    }
//    if(fabs(ori + PI / 2) < eps) {
//        work(0); work(1); work_in_row(1);
//    }
//    if(fabs(ori) < eps) {
//        work(1); work(2); work_in_row(2);
//    }
//    if(fabs(ori - PI / 2) < eps) {
//        work(2); work(3); work_in_row(3);
//    }
//}