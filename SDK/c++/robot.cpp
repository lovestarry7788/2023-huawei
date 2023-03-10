//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"
#include <iostream>
#include <algorithm>

Robot::Robot(size_t id, size_t workbench, size_t carry_id, double time_coefficient, double collide_coefficient,
             double angular_velocity, double linear_velocity, double orient, double x0, double y0) :
             id_(id), workbench_(workbench), carry_id_(carry_id), time_coefficient_(time_coefficient), collide_coefficient_(collide_coefficient),
             angular_velocity_(angular_velocity), linear_velocity_(linear_velocity), orient_(orient), x0_(x0), y0_(y0){}

