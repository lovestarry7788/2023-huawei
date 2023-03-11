//#include "workbench.h"
//#include "robot.h"
//#include "Init.h"
//#include "map_.h"
#include "solution1.h"
#include <iostream>
#include <vector>
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
        for(int i = 0; i < K; ++i) {
            size_t type_id, frame_remain, materials_status, product_status;
            double x0, y0;
            scanf("%zd%lf%lf%zd%zd%zd",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
            workbench[i] = make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
        }

        robot.resize(4);
        for(size_t id = 0; id < 4; ++id) {
            size_t workbench, carry_id;
            double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
            scanf("%zd%zd%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                  &orient, &x0, &y0);
            robot[id] = make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
        }

        readUntilOK();

        solve(frameID, coins, K, workbench, robot);

//        outputOK(frameID);
    }
    return 0;
}