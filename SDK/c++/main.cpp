//#include "workbench.h"
//#include "robot.h"
//#include "map_.h"
#include "solution1.h"
#include <iostream>
#include <algorithm>

using namespace std;

int main() {
    map_ Map;
    Map.init();

    //地图输入结束

    std::vector<std::shared_ptr<Workbench> > workbench;
    std::vector<std::shared_ptr<Robot> > robot;
    int frameID, coins;
    while (scanf("%d%d", &frameID, &coins) != EOF) {
        int K;
        scanf("%d",&K);
        workbench.resize(K);
        for(int i = 0; i < K; ++i)
            workbench[i]->init();

        robot.resize(4);
        for(int id = 0; id < 4; ++id)
            robot[id]->init(id);

        readUntilOK();

        solve(frameID, coins, K, workbench, robot);

        outputOK(frameID);
//        break;
    }
    return 0;
}
