//
// Created by 刘智杰 on 2023/3/10.
//
#include "robot.h"

Robot::Robot(int id, int workbench, int carry_id, double time_coefficient, double collide_coefficient,
             double angular_velocity, double x_velocity, double y_velocity, double orient, double x0, double y0) :
             id_(id), workbench_(workbench), carry_id_(carry_id), time_coefficient_(time_coefficient), collide_coefficient_(collide_coefficient),
             angular_velocity_(angular_velocity), linear_velocity_(sqrt(x_velocity * x_velocity + y_velocity * y_velocity)), orient_(orient), x0_(x0), y0_(y0){}

void Robot::ToPoint(double x0, double y0, double& forward, double& rotate) {
    double angle = atan2((y0 - y0_) , (x0 - x0_)); // 计算到目标点的弧度
    /*
     * orient_, angle [-pi, pi]
     * 20ms, pi / s
    */
    double delta_angle = (angle - orient_);
    if(fabs(delta_angle) > acos(-1)) {
        delta_angle = (2 * acos(-1) - fabs(delta_angle)) * (-1);
    }

    if(fabs(delta_angle) >= 0.02 * max_rotate_velocity_) {
        rotate = max_rotate_velocity_;
    } else {
        rotate = delta_angle / 0.02;
    }

    // todo：目前只考虑了一直加速往前冲（方向相反也加速往前冲）
    double distance = sqrt((x0_ - x0) * (x0_ - x0) + (y0_ - y0) * (y0_ - y0));
    if(distance >= 0.02 * max_forward_velocity_) {
        forward = max_forward_velocity_;
    } else {
        forward = distance / 0.02;
    }
}