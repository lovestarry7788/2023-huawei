//
// Created by gllonkxc on 2023/3/10.
//
#ifndef HW2023_INIT_H
#define HW2023_INIT_H

#pragma once
#include "robot.h"
#include "workbench.h"
#include <iostream>
#include <cmath>
#include <memory>
using namespace std;

struct Init{
    char map_[105][105];
    bool readUntilOK() {
        char line[1024];
        while (fgets(line, sizeof line, stdin)) {
            if (line[0] == 'O' && line[1] == 'K') {
                return true;
            }
        }
        return false;
    }
    inline void Init_map() {
        for(int i = 0; i < 100; ++i) {
            for(int j = 0; j < 100; ++j) {
                scanf("\n%c",&map_[i][j]);
            }
        }
        readUntilOK();
    }
//    Workbench workbench[51];
//    inline void Init_workbench(Workbench &workbench) {
//        size_t type_id, frame_remain, materials_status, product_status;
//        double x0, y0;
//        scanf("%zd%lf%lf%zd%zd%zd",&type_id, &x0, &y0, &frame_remain, &materials_status, &product_status);
//        workbench = make_shared<Workbench>(type_id, x0, y0, frame_remain, materials_status, product_status);
//    }
//    inline void Init_Robot(Robot robot, int id) {
//        size_t workbench, carry_id;
//        double time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0;
//        scanf("%zd%zd%lf%lf%lf%lf%lf%lf%lf",&workbench, &carry_id, &time_coefficient, &collide_coefficient, &angular_velocity, &linear_velocity,
//              &orient, &x0, &y0);
//        robot = make_shared<Robot>(id, workbench, carry_id, time_coefficient, collide_coefficient, angular_velocity, linear_velocity, orient, x0, y0);
//    }
};

#endif
