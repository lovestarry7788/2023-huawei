#include "input.h"
#include "log.h"
#include "IDA.h"

int Input::frameID, Input::coins, Input::K;
char Input::map_[101][101];
int Input::map_number_;
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
    K = 0;
    // 地图输入开始
    for(int i = 0; i < 100; ++i) {
        for(int j = 0; j < 100; ++j) {
            scanf("\n%c",&map_[i][j]);
            K += map_[i][j] >= '1' && map_[i][j] <= '9'; 
            // Log::print("i: ", i, "j: ", j, map_[i][j]);
            if(map_[i][j] == '#') is_obstacle_[i][j] = true;
            else is_obstacle_[i][j] = false;
        }
    }
    readUntilOK();
    puts("OK");
    fflush(stdout);
    Identify_Map_Number();
    IDA::Init();
}

bool Input::ScanFrame() { 
    while (scanf("%d%d", &frameID, &coins) != EOF) {
        scanf("%d",&K);
        workbench.resize(K);
        for(int i = 0; i < K; ++i) {
            int type_id, frame_remain, materials_status, product_status;
            double x0, y0;
            scanf("%d%lf%lf%d%d%d",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
            // Log::print(frameID, coins, type_id, x0, y0, frame_remain, materials_status, product_status);
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
            // Log::print(workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity_x, linear_velocity_y, orient, x0, y0);
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

void Input::Identify_Map_Number() {
    if(map_[1][49] == '1' && map_[98][1] == '2' && map_[98][98] == '3') map_number_ = 1;
    else if(map_[1][1] == '6' && map_[1][49] == '5' && map_[1][98] == '6') map_number_ = 2;
    else if(map_[1][46] == '3' && map_[1][54] == '5') map_number_ = 3;
    else if(map_[7][49] == '7' && map_[97][49] == '4') map_number_ = 4;
    else map_number_ = -1;
    Log::print("Identify_Map_Number: ", map_number_);
}


void Input::Planned_Route() {

    for(int idx = 0; idx < robot_num_ + K; ++idx) {
        for(int idy = 0; idy < robot_num_ + K; ++idy) {
            double sx, sy, dx, dy;
            std::vector<Geometry::Point> planned_route;
            if(idx < 4) {
                sx = robot[idx] -> x0_;
                sy = robot[idx] -> y0_;
            } else {
                sx = workbench[idx - robot_num_] -> x0_;
                sy = workbench[idx - robot_num_] -> y0_;
            }
            if(idy < 4) {
                dx = robot[idy] -> x0_;
                dy = robot[idy] -> y0_;
            } else {
                dx = workbench[idy - robot_num_] -> x0_;
                dy = workbench[idy - robot_num_] -> y0_;
            }

            swap(planned_route_[idx][idy], planned_route);
        }
    }

}