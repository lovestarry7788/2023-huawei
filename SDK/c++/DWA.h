//
// Created by gllonkxc on 2023/3/16.
//

#ifndef CODECRAFTSDK_DWA_H
#define CODECRAFTSDK_DWA_H

#include "geometry.h"
//#include "Point.h"
#include <cmath>
#include "Environment.h"
#include <vector>
#include <iostream>
using namespace std;
using namespace Geometry;

const double pi = acos(-1);

struct Car
{
    double max_speed = 6;
    double min_speed = -2;
    double max_angular_speed = pi;
    double min_angular_speed = -pi;

    const double J = 50;
    const double F = 250;

    double radius = 0.42;

    double m = 20 * pi * radius * radius;
    double M = 0.5 * pi * radius * radius;

    double max_accel = F / m;
    double max_angular_speed_rate = J / M;//角加速度 J/M
    double v_resolution = 0.02;     // 速度采样分辨率
    double yaw_rate_resolution = 0.1 * pi / 180;
    double dt = 0.02;
    double predict_time = 2.0;  //运动学模型预测时间
    double goal_cost_gain = 0.2;
    double speed_cost_gain = 1.0;
    double obstacle_cost_gain = 1.0;

};//robot[i]->

struct CarState
{
    double x;
    double y;
    double yaw;
    double speed;
    double angular_speed;
    CarState(){};
    CarState(double x_, double y_, double yaw_, double speed_, double angular_speed_):
            x(x_), y(y_), yaw(yaw_), speed(speed_), angular_speed(angular_speed_){}
};//robot[i]->

class DWA{
public:

    DWA();
//    DWA(Environment* env, Geometry::Point start, Geometry::Point destination);
    DWA(Car car, vector<vector<CarState>> trajectory,
        Geometry::Point startPoint,
        Geometry::Point destinationPoint,
        CarState currentState,
        CarState destinationState,
        Environment *environment, bool aaa);
    vector<double> planning();
    vector<double> dwa_control(const CarState &carstate);
    vector<double> calc_dw(const CarState &carstate);
    vector<double> calc_best_speed(const CarState &carstate, const vector<double> &dw);
    void predict_trajectory(const CarState &carstate, const double &speed, const double &angular_speed, vector<CarState> &trajectory);
    CarState motion_model(const CarState &carstate, const double &speed, const double &angular_speed);
    double calc_goal_cost(const vector<CarState> &trajectory);
    double calc_obstacle_cost(const vector<CarState> &trajectory);

    Car car;
    vector<vector<CarState>> trajectory;
    Geometry::Point startPoint;
    Geometry::Point destinationPoint;
    CarState currentState;
    CarState destinationState;
    Environment *environment;
    bool aaa = false;
};

#endif //CODECRAFTSDK_DWA_H
