#include "wayfinding.h"
#include <algorithm>

WayFindding::WayFindding(char (*map)[100]): map_(map) {
    for (int i = 0; i < 100; i++) for (int j = 0; j < 100; j++) {
        if (map[i][j] != '#') continue;
        for (auto [di, dj] : {{1, -1},{1, 1},{-1, -1},{-1, 1}}) {
            int ni = di+i, nj = dj+j;
            if (ni < 0 || ni >= 100 || nj < 0 || nj >= 100) continue;
            if (!(map[ni][nj] != '#' || map[ni][j] != '#' || map[i][nj] != '#')) continue;
            joint_.push_back({ni, nj});
        }
    }
    std::sort(begin(joint_), end(joint_));
    joint_.resize(std::unique(begin(joint_), end(joint_)) - begin(joint_));
}

void WayFindding::dijkstra(int s) {

}

void WayFindding::calc_all_route() {

}