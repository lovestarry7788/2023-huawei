//
// Created by 刘智杰 on 2023/3/10.
//

#include "input.h"
#include "robot.h"
#include "workbench.h"
#include <vector>


bool Input::readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
    }
    return false;
}

void Input::ScanMap(char **map_) {
    // 地图输入开始
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j < 100; ++j) {
            scanf("\n%c",&map_[i][j]);
        }
    }
    readUntilOK();
    puts("OK");
    fflush(stdout);
}

void Input::ScanFrame(int* frameID, int* coins, int *K, std::vector<std::shared_ptr<Workbench> > workbench, std::vector<std::shared_ptr<Robot> > robot) {
    while (scanf("%d%d", frameID, coins) != EOF) {
        int K;
        scanf("%d",&K);
        workbench.resize(K);
        for(int i = 0; i < K; ++i) {
            int type_id, frame_remain, materials_status, product_status;
            double x0, y0;
            scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
            workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
        }

        robot.resize(4);
        for(int id = 0; id < 4; ++id) {
            int workbench, carry_id;
            double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
            scanf("%d%d%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
                  &orient, &x0, &y0);
            robot[id] = std::make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
        }

        readUntilOK();
        break;
    }
}
