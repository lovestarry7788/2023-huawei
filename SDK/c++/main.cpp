#include "workbench.h"
#include "robot.h"
#include <iostream>
#include <algorithm>
#include <memory>


using namespace std;

bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

char map_[101][101];

int main() {

    // 地图输入开始
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j < 100; ++j) {
            scanf("\n%c",&map_[i][j]);
        }
    }
    readUntilOK();

    puts("OK");
    fflush(stdout);
    // 地图输入结束

    shared_ptr<Workbench> workbench[51];
    shared_ptr<Robot> robot[4];
    int frameID, coins;
    while (scanf("%d%d", &frameID, &coins) != EOF) {
        int K;
        scanf("%d",&K);
        for(int i = 1; i <= K; ++i) {
            int type_id, frame_remain, materials_status, product_status;
            double x0, y0;
            scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
            workbench[i] = make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
        }
        for(int id = 0; id < 4; ++id) {
            int workbench, carry_id;
            double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
            scanf("%d%d%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                  &orient, &x0, &y0);
            robot[id] = make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
        }

        readUntilOK();

        /* Solution */
        printf("%d\n", frameID);
        puts("OK");
        fflush(stdout);
    }
    return 0;
}
