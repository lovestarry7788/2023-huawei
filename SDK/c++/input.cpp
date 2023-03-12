#include "input.h"

int Input::frameID, Input::coins, Input::K;
char Input::map_[101][101];
std::vector<std::shared_ptr<Workbench> > Input::workbench;
std::vector<std::shared_ptr<Robot> > Input::robot;        

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

void Input::ScanMap() {
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

bool Input::ScanFrame() { 
    while (scanf("%d%d", &frameID, &coins) != EOF) {
        scanf("%d",&K);
        workbench.resize(K);
        for(int i = 0; i < K; ++i) {
            int type_id, frame_remain, materials_status, product_status;
            double x0, y0;
            scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
            if (!workbench[i])
                workbench[i] = std::make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
            else {
                workbench[i]->type_id_ = type_id;
                workbench[i]->x0_ = x0;
                workbench[i]->y0_ = y0;
                workbench[i]->frame_remain_ = frame_remain;
                workbench[i]->materials_status_ = materials_status;
                workbench[i]->product_status_ = product_status;
            }
        }

        robot.resize(4);
        for(int id = 0; id < 4; ++id) {
            int workbench, carry_id;
            double time_coefficient, collide_coefficient, angular_velocity, linear_velocity_x, linear_velocity_y, orient, x0, y0;
            scanf("%d%d%lf%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity_x, &linear_velocity_y, 
                    &orient, &x0, &y0);
            if (!robot[id]) 
                robot[id] = std::make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity_x, linear_velocity_y, orient, x0, y0);
            else {
                robot[id]->id_ = id;
                robot[id]->workbench_ = workbench;
                robot[id]->carry_id_ = carry_id;
                robot[id]->time_coefficient_ = time_coefficient;
                robot[id]->collide_coefficient_ = collide_coefficient;
                robot[id]->angular_velocity_ = angular_velocity;
                robot[id]->linear_velocity_x_ = linear_velocity_x;
                robot[id]->linear_velocity_y_ = linear_velocity_y;
                robot[id]->orient_ = orient;
                robot[id]->x0_ = x0;
                robot[id]->y0_ = y0;
            }
        }

        readUntilOK();
        return true;
    }
    return false;
}
